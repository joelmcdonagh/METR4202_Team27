import rclpy
from exploration import Exploration

def start_navigation(args):
    rclpy.init(args=args)
    node = Exploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    start_navigation(args)

if __name__ == '__main__':
    main()
