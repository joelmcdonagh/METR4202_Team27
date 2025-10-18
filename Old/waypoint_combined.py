#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import numpy as np



# Recovery

class Recovery:
    """Handles recovery actions and keeps a blacklist of failed frontiers."""
    def __init__(self, parent_node):
        self.node = parent_node
        self.blacklist = []

    def recover(self, last_goal):
        """Add the failed goal to the blacklist."""
        if last_goal is not None:
            rounded = self.node.round_coords(last_goal)
            self.blacklist.append(rounded)
            self.node.get_logger().warn(f"Added {rounded} to blacklist during recovery.")
        else:
            self.node.get_logger().warn("Recovery triggered but no goal provided.")



# Exploration CLASS

class Exploration(Node):
    def __init__(self):
        super().__init__('exploration_node')

        # ---- Subscribers ----
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # ---- Nav2 client ----
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()

        # ---- State ----
        self.recovery = Recovery(self)
        self.occupancy_grid = None
        self.map_info = None
        self.frontier_cells = []
        self.visited_frontiers = []
        self.current_goal = None
        self.robot_pos = (0.0, 0.0)

        self.get_logger().info("Exploration node initialized.")

    
    # Utility
    
    def round_coords(self, coord, decimals=2):
        return (round(coord[0], decimals), round(coord[1], decimals))

    
    # ODOM callback
    
    def odom_callback(self, msg):
        """Track robot position and goal selection logic."""
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # If no current goal, try to pick one
        if self.current_goal is None and self.frontier_cells:
            self.get_logger().info("Choosing new frontier...")
            chosen_frontier = self.pick_frontier(self.frontier_cells, self.robot_pos)
            if chosen_frontier:
                self.get_logger().info(f"Chosen frontier: {chosen_frontier}")
                self.current_goal = chosen_frontier

                if len(self.visited_frontiers) > 100:
                    self.visited_frontiers.pop(0)
                self.visited_frontiers.append(self.round_coords(chosen_frontier))

                self.send_goal(*chosen_frontier)

        # If near goal, reset
        elif self.current_goal is not None:
            dist = np.hypot(self.robot_pos[0] - self.current_goal[0],
                            self.robot_pos[1] - self.current_goal[1])
            if dist < 0.4:
                self.get_logger().info("Goal reached!")
                self.current_goal = None

    
    # MAP callback
    
    def map_callback(self, msg: OccupancyGrid):
        """Convert OccupancyGrid data into numpy and detect frontiers."""
        new_grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.occupancy_grid = new_grid
        self.map_info = msg.info
        self.frontier_cells = self.find_frontiers()

        self.get_logger().info(f"Map updated — {len(self.frontier_cells)} frontier cells found.")

    
    # FRONTIER detection
    
    def find_frontiers(self):
        if self.occupancy_grid is None:
            return []

        frontiers = []
        grid = self.occupancy_grid
        h, w = grid.shape

        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if grid[y, x] != 0:
                    continue
                # Neighbours up/down/left/right
                n = [grid[y + 1, x], grid[y - 1, x], grid[y, x + 1], grid[y, x - 1]]
                if -1 in n:
                    frontiers.append((y, x))
        return frontiers

    
    # PICK frontier
    
    def pick_frontier(self, frontiers, robot_pos):
        if not frontiers:
            return None
        res = self.map_info.resolution

        def grid_to_world(y, x):
            wx = self.map_info.origin.position.x + (x + 0.5) * res
            wy = self.map_info.origin.position.y + (y + 0.5) * res
            return (wx, wy)

        unvisited = [
            f for f in frontiers
            if self.round_coords(grid_to_world(*f)) not in self.visited_frontiers
            and self.round_coords(grid_to_world(*f)) not in self.recovery.blacklist
        ]
        if not unvisited:
            self.get_logger().info("All frontiers visited or blacklisted.")
            return None

        closest = min(
            unvisited,
            key=lambda f: np.hypot(grid_to_world(*f)[0] - robot_pos[0],
                                   grid_to_world(*f)[1] - robot_pos[1])
        )
        return grid_to_world(*closest)

    
    # NAV2 Goal Sending
    
    def send_goal(self, x, y):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal.pose = pose

        self.get_logger().info(f"Sending goal: ({x:.2f}, {y:.2f})")
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            self.current_goal = None
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().warn("Goal aborted. Triggering recovery.")
            self.recovery.recover(self.current_goal)
            self.current_goal = None
        elif status == 5:
            self.get_logger().warn("Goal cancelled.")
            self.current_goal = None
        else:
            self.get_logger().info("Goal succeeded.")
            self.current_goal = None



# MAIN ENTRY POINT

def main(args=None):
    rclpy.init(args=args)
    node = Exploration()
    node.get_logger().info("Exploration node running.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exploration stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
