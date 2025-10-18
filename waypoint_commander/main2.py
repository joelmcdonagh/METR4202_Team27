import rclpy
from rclpy.node import Node
class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')
        self.get_logger().info('Exploration node started')
def main():
    rclpy.init()
    node = ExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
