"""Navigation node, one instance per robot namespace."""
import rclpy
from rclpy.node import Node


class RobotHandler(Node):

    def __init__(self):
        super().__init__('robot_handler')
        self.get_logger().info('robot_handler started')


def main(args=None):
    rclpy.init(args=args)
    node = RobotHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
