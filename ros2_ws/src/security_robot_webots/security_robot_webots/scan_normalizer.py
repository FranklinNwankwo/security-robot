import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanNormalizer(Node):
    """Republishes /scan_raw as /scan with a positive angle_increment (counter-clockwise),
    which is what SLAM Toolbox and Nav2 expect."""

    def __init__(self):
        super().__init__('scan_normalizer')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan_raw', self.callback, 10)
        self.reported = False

    def callback(self, msg):
        if msg.angle_increment < 0.0:
            out = LaserScan()
            out.header = msg.header
            out.angle_min = msg.angle_max
            out.angle_max = msg.angle_min
            out.angle_increment = -msg.angle_increment
            out.time_increment = msg.time_increment
            out.scan_time = msg.scan_time
            out.range_min = msg.range_min
            out.range_max = msg.range_max
            out.ranges = list(reversed(msg.ranges))
            out.intensities = list(reversed(msg.intensities))
        else:
            out = msg

        if not self.reported:
            self.get_logger().info(
                f'raw angle_increment={msg.angle_increment:.5f} -> '
                f'{"reversed to counter-clockwise" if msg.angle_increment < 0.0 else "already counter-clockwise, passing through"}'
            )
            self.reported = True

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
