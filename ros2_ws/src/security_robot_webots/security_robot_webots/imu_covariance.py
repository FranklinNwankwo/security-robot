"""Adds realistic uncertainty to the simulated IMU. Webots publishes all-zero covariance ("perfect sensor"),
which makes the EKF trust the gyro blindly. Real IMU drivers report these values; this node does the same."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

GYRO_VAR = 0.02 ** 2      # (rad/s)^2  - typical MEMS gyro
ACCEL_VAR = 0.2 ** 2      # (m/s^2)^2
ORIENT_VAR = 0.05 ** 2    # rad^2      - published, but not fused by the EKF


def diag(v):
    return [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v]


class ImuCovariance(Node):
    def __init__(self):
        super().__init__('imu_covariance')
        self.pub = self.create_publisher(Imu, '/imu', 10)
        self.create_subscription(Imu, '/imu/raw', self.callback, qos_profile_sensor_data)
        self.get_logger().info('Adding realistic covariance: /imu/raw -> /imu')

    def callback(self, msg):
        msg.orientation_covariance = diag(ORIENT_VAR)
        msg.angular_velocity_covariance = diag(GYRO_VAR)
        msg.linear_acceleration_covariance = diag(ACCEL_VAR)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovariance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
