import rclpy
from .exploration import Exploration
from rclpy.executors import MultiThreadedExecutor

def start_navigation(args):
    rclpy.init(args=args)

    node = Exploration()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Navigation stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    start_navigation(args)
