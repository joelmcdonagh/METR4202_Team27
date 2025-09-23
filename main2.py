import rclpy
from exploration import Exploration
from recovery import Recovery   

def start_navigation(args):
    rclpy.init(args=args)

    node = Exploration()
    
    node.recovery = Recovery(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    start_navigation(args)

if __name__ == '__main__':
    main()
