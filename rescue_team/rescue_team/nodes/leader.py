"""Global leader: mission state machine, reporting and goal allocation."""
import rclpy
from rclpy.node import Node


class Leader(Node):

    def __init__(self):
        super().__init__('leader')
        self.get_logger().info('leader started')


def main(args=None):
    rclpy.init(args=args)
    node = Leader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
