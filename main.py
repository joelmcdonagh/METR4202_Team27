import rclpy
from waypoint_cycler_recovery import WaypointRecoveryCommander

def start_navigation(args):
    rclpy.init(args=args)
    node = WaypointRecoveryCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    start_navigation(args)

if __name__ == '__main__':
    main()
