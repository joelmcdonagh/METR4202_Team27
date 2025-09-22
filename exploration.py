import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import numpy as np

class Exploration(Node):

    def __init__(self):
        # Subscribe to occupancy grid
        super().__init__('map_listener')
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 5)
        self.occupancy_grid = None
        self.map_info = None
        self.frontier_cells = []
        self.current_goal = None

        # Subscribe to pose
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 5)
        self.robot_pos = (0, 0)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Wait for server if ros2 isnt open
        self.nav_client.wait_for_server()

    def pose_callback(self, msg):
        # Update robot position
        self.robot_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        # Track goal
        if self.current_goal is not None:
            dist = np.hypot(self.robot_pos[0]-self.current_goal[0], self.robot_pos[1]-self.current_goal[1])
            if dist < 0.7:  # 30cm tolerance from goal
                self.get_logger().info("Goal reached!")
                self.current_goal = None

                # Pick a new frontier
                if self.frontier_cells:
                    chosen_frontier = self.pick_frontier(self.frontier_cells, self.robot_pos)
                    if chosen_frontier:
                        self.get_logger().info(f"Chosen frontier is {chosen_frontier}")

                        # Set current goal and send to nav2
                        self.current_goal = chosen_frontier
                        self.send_goal(*chosen_frontier)


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
        
        # Currently just find closest frontier --- CHANGE
        closest = min(frontiers, key=lambda f: np.hypot(get_world_coords(*f)[0]-robot_position[0],
                                                     get_world_coords(*f)[1]-robot_position[1]))
        return get_world_coords(*closest)
    
    def goal_receive_callback(self, future):
        goal_h = future.result()
        if not goal_h.accepted:
            self.get_logger().info("Goal rejected")
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
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal).add_done_callback(self.goal_receive_callback)
    

    




