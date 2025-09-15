import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class Exploration(Node):

    def __init__(self):
        # Subscribe to occupancy grid
        super.__init__('map_listener')
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 5)
        self.occupancy_grid = None

    def map_callback(self, msg: OccupancyGrid):
        # Convert data array from SLAM into numpy 2D array
        new_grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        # Update occupancy grid
        self.occupancy_grid = new_grid
        self.get_logger.info("Map Updated")

        # Move onto detecting cells with unkown neighbours
        self.frontier_cells = self.find_frontiers()

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

    def pick_frontier(frontiers: list, robot_position: tuple, origin: tuple, grid_res: float):
        if not frontiers:
            return None
        
        # Function to convert cell height/width in grid to real-world coordinates
        def get_world_coords(h, w):
            x = origin[0] + (w + 0.5) * grid_res
            y = origin[1] + (h + 0.5) * grid_res
            return (x, y)
        
        # Currently just find closest frontier --- CHANGE
        closest = min(frontiers, key=lambda f: np.hypot(get_world_coords(*f)[0]-robot_position[0],
                                                     get_world_coords(*f)[1]-robot_position[1]))
        return get_world_coords(*closest)
    

    




