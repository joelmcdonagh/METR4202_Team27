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
import time
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

        # Initialise the goal_time and frontier_update time counter to slow updates
        self.goal_time = 0.0
        self.frontier_update_time = 0.0

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

        # Get current time for goal timer
        current_time = time.time()

        # If there is already a goal in progress, track current goal
        if self.current_goal is not None:

            # First check if the goal has been reached
            dist = np.hypot(self.robot_pos[0]-self.current_goal[0], self.robot_pos[1]-self.current_goal[1])
            if dist < 0.4:  # 40cm tolerance from goal
                # If goal reached, set current_goal to None so another will be found
                self.get_logger().info("Goal reached!")
                self.current_goal = None
                self.goal_time = 0.0
                return
            
            # Return if goal time limit (30 seconds) has not been reached yet
            if current_time - self.goal_time < 30.0:
                return
            
            # If goal timer expired, reset
            self.get_logger().info("Goal timed out - Starting new goal")
            self.current_goal = None
            self.goal_time = 0.0

        # If the frontier list is empty and there is no current goal, find and send a new goal
        if self.current_goal is None and self.frontier_cells:

            # Cluster frontiers
            frontier_clusters = self.cluster_frontiers(self.frontier_cells, 4)
            self.get_logger().info(f"Generated {len(frontier_clusters)} clusters")

            # Choose frontier cluster
            chosen_frontier = self.pick_frontier_cluster(frontier_clusters, self.robot_pos)

            if chosen_frontier:
                # Frontier has been chosen
                self.get_logger().info(f"Chosen cluster is {chosen_frontier}")
                self.current_goal = chosen_frontier
                self.goal_time = current_time

                # Shorten oldest frontier in visited frontiers list so it doesn't grow too high
                if len(self.visited_frontiers) > 100:
                    self.visited_frontiers.pop(0)
                
                # Add to visited list, but keep the list small
                self.visited_frontiers.append(self.round_coords(chosen_frontier))

                # Send goal to NAV2
                self.send_goal(*chosen_frontier)
    
    def map_callback(self, msg: OccupancyGrid):
        # Convert data array from SLAM into numpy 2D array
        new_grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        # Update occupancy grid
        self.occupancy_grid = new_grid
        self.map_info = msg.info

        # Timer to limit finding frontiers to every 3 seconds
        current_time = time.time()
        if current_time - self.frontier_update_time < 5.0:
            return  # Return if timer hasnt been reached
        
        # If timer reached, update map
        self.frontier_update_time = current_time
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

    def cluster_frontiers(self, frontiers: list, distance_threshold):

        # Helper function to get a neighbours list of given cell
        def get_neighbours_of_cell(cell):
            height, width = cell
            neighbours = []
            neighbours.append((height - 1, width)) # Append neighbour above cell
            neighbours.append((height + 1, width)) # Append neighbour below cell
            neighbours.append((height, width - 1)) # Append neighbour to left of cell
            neighbours.append((height, width + 1)) # Append neighbour to right of cell

            # Diagonals
            neighbours.append((height - 1, width - 1))
            neighbours.append((height - 1, width + 1))
            neighbours.append((height + 1, width - 1))
            neighbours.append((height + 1, width + 1))

            return neighbours
        
        clusters = []
        visited_frontiers = set()

        # Loop through all frontiers in given list
        for frontier in frontiers:
            # Ignore frontier if it has been visited
            if frontier in visited_frontiers:
                continue

            current_cluster = []
            queue = [frontier]
            visited_frontiers.add(frontier)

            # Use Breadth-first-search to cluster frontiers
            while queue:
                current_frontier = queue.pop(0)
                current_cluster.append(current_frontier)

                # Check for neighbours around this cell which are also frontier cells
                for neighbour in get_neighbours_of_cell(current_frontier):
                    if neighbour in frontiers and neighbour not in visited_frontiers:
                        # Add it to the cluster if distance is less than threshold (radius of clustering)
                        if abs(neighbour[0] - current_frontier[0]) <= distance_threshold and abs(neighbour[1] - current_frontier[1]) <= distance_threshold:
                            visited_frontiers.add(neighbour)
                            queue.append(neighbour)

            # Add current cluster to list
            if current_cluster:
                clusters.append(current_cluster)

        return clusters


        


    def pick_frontier_cluster(self, frontier_clusters: list, robot_position: tuple):
        if not frontier_clusters:
            return None
    
        grid_res = self.map_info.resolution

        # convert cell height/width in grid to real-world coordinates
        def get_world_coords(h, w):
            x = self.map_info.origin.position.x + (w + 0.5) * grid_res
            y = self.map_info.origin.position.y + (h + 0.5) * grid_res
            return (x, y)

        # Make centroids
        centroids = []
        for cluster in frontier_clusters:
            average_height_cluster = sum(c[0] for c in cluster) / len(cluster)
            average_width_cluster = sum(c[1] for c in cluster) / len(cluster)
            centroids.append(get_world_coords(average_height_cluster, average_width_cluster))
        
        # Filter out visited and blacklisted clusters
        unvisited = [c for c in centroids
                    if self.round_coords(c) not in self.visited_frontiers
                    and self.round_coords(c) not in self.recovery.blacklist]
        if not unvisited:
            self.get_logger().info("Every frontier has been visited or is blacklisted")
            return None
        
        # Function for getting distance to nearest wall
        def wall_distance(world_point):
            grid = self.occupancy_grid
            height, width = grid.shape
            x, y = world_point

            # convert back to grid coords
            grid_X = int((x - self.map_info.origin.position.x) / grid_res)
            grid_Y = int((y - self.map_info.origin.position.y) / grid_res)

            min_dist = float("inf")
            for h in range(max(0, grid_Y-10), min(height, grid_Y+10)):
                for w in range(max(0, grid_X-10), min(width, grid_X+10)):
                    if grid[h, w] == 100:  # wall cell
                        dist = np.hypot(h-grid_Y, w-grid_X) * grid_res
                        if dist < min_dist:
                            min_dist = dist
            return min_dist

        # Use the best centroid by finding score
        def get_cluster_score(centroid_pos):
            # Use distance to formula to find distance between cluster centroid and robot
            distance_to_robot = np.hypot(centroid_pos[0]-robot_position[0], centroid_pos[1]-robot_position[1])
            # Use wall distance helper function to find distance from centroid to wall
            #distance_to_wall = wall_distance(centroid_pos)

            # Return score
            return distance_to_robot

        # Choose the best centroid which has not been explored yet using score function
        best_goal = min(unvisited, key=get_cluster_score)
        return best_goal

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

        
    

    




