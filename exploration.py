import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
from .recovery import Recovery

class Exploration(Node):

    def __init__(self):
        # Subscribe to occupancy grid
        super().__init__('map_listener')
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Subscribe to odometry
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.occupancy_grid = None
        self.map_info = None
        self.frontier_cells = []
        self.visited_frontiers = [] # Start a list of blacklisted frontiers to not visit again
        self.current_goal = None

        # Subscribe to pose
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.robot_pos = (0, 0)

        # Wait for server if ros2 isnt open
        self.nav_client.wait_for_server()

        # Instantiate recovery class
        self.recovery = Recovery(self)

    def round_coords(self, coord, decimals=2):
        return (round(coord[0], decimals), round(coord[1], decimals))

    def odom_callback(self, msg):
        # Update robot position
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # If no current goal, try to pick one
        if self.current_goal is None and self.frontier_cells:
            self.get_logger().info(f"Choosing frontier")
            chosen_frontier = self.pick_frontier(self.frontier_cells, self.robot_pos)
            if chosen_frontier:
                self.get_logger().info(f"Chosen frontier is {chosen_frontier}")
                self.current_goal = chosen_frontier

                # Shorten oldest frontier in visited frontiers list so it doesn't grow too high
                if len(self.visited_frontiers) > 100:
                    self.visited_frontiers.pop(0)
                
                # Add to visited list
                self.visited_frontiers.append(self.round_coords(chosen_frontier))

                # Send goal to NAV2
                self.send_goal(*chosen_frontier)

        # Otherwise, track the current goal
        elif self.current_goal is not None:
            # Check distance to goal
            dist = np.hypot(self.robot_pos[0]-self.current_goal[0], self.robot_pos[1]-self.current_goal[1])
            if dist < 0.4:  # 40cm tolerance from goal
                # If goal reached, set current_goal to None so another will be found
                self.get_logger().info("Goal reached!")
                self.current_goal = None

    def map_callback(self, msg: OccupancyGrid):
        # Convert data array from SLAM into numpy 2D array
        new_grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        # Update occupancy grid
        self.occupancy_grid = new_grid
        self.map_info = msg.info
        self.get_logger().info("Map Updated")

        # Move onto detecting cells with unkown neighbours
        self.frontier_cells = self.find_frontiers()
        self.get_logger().info(f"Found {len(self.frontier_cells)} frontier cells")

    #def timer_callback(self):
    #    # Force a new goal every 10 seconds regardless
    #    if self.current_goal is not None:
    #        self.get_logger().info("Forcing new goal after timeout")
    #        self.current_goal = None

    def find_frontiers(self):
        if self.occupancy_grid is None:
            return []
        
        frontiers = []
        grid = self.occupancy_grid
        height, width = grid.shape

        # Check each cell in occupancy grid (subtract 1 from height/width so edges are not checked)
        for h in range(1, height-1):
            for w in range(1, width-1):
                # Check if the current cell is already known. If it is, skip it
                if grid[h, w] != 0:
                    continue
                # Grab neighbours of current cell
                cell_neighbours = [grid[h+1, w], grid[h, w+1], grid[h-1, w], grid[h, w-1]]
                if -1 in cell_neighbours:
                    frontiers.append((h, w))
        return frontiers

    def pick_frontier(self, frontiers: list, robot_position: tuple):
        if not frontiers:
            return None
    
        grid_res = self.map_info.resolution

        # convert cell height/width in grid to real-world coordinates
        def get_world_coords(h, w):
            x = self.map_info.origin.position.x + (w + 0.5) * grid_res
            y = self.map_info.origin.position.y + (h + 0.5) * grid_res
            return (x, y)
        
        # Filter out visited frontiers and blacklisted cells
        unvisited = [f for f in frontiers
                    if self.round_coords(get_world_coords(*f)) not in self.visited_frontiers
                    and self.round_coords(get_world_coords(*f)) not in self.recovery.blacklist]
        if not unvisited:
            self.get_logger().info("Every frontier has been visited or is blacklisted")
            return None


        # Currently just find closest frontier --- CHANGE
        closest = min(unvisited, key=lambda f: np.hypot(get_world_coords(*f)[0]-robot_position[0],
                                                     get_world_coords(*f)[1]-robot_position[1]))
        return get_world_coords(*closest)

    def goal_receive_callback(self, future):
        goal_h = future.result()
        if not goal_h.accepted:
            self.get_logger().info("Goal rejected")
            self.current_goal = None
            return
        self.get_logger().info("Goal accepted")

    def send_goal(self, x, y):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # facing forward
        goal.pose = pose

        self.get_logger().info(f"Sending Nav2 goal: ({x:.2f}, {y:.2f})")

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal = future.result()
        if not goal.accepted:
            self.get_logger().warn("Goal rejected")
            self.current_goal = None
            return

        self.get_logger().info("Goal accepted, waiting for result")
        result_future = goal.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status

        # Check goal result and report back to user
        if status == 4:  # Goal was aborted
            self.get_logger().warn("Goal aborted, triggering recovery")
            self.recovery.recover(self.current_goal)
            self.current_goal = None
        elif status == 5:  # Goal was canceled
            self.get_logger().warn("Goal canceled")
            self.current_goal = None
        else: # Goal was successful
            self.get_logger().info("Goal succeeded")
            self.current_goal = None

        
    

    




