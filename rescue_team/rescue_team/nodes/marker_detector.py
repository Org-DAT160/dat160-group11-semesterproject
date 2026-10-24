"""Global marker detector: ArUco detections to map-frame sightings."""
import rclpy
from rclpy.node import Node


class MarkerDetector(Node):

    def __init__(self):
        super().__init__('marker_detector')
        self.get_logger().info('marker_detector started')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
