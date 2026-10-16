import math

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

WHEEL_RADIUS = 0.03            # m  - must match the wheel Cylinder radius in the world file
HALF_WHEEL_SEPARATION = 0.10   # m  - effective (calibrated): wheel contact is at the outer edge, not the centre
MAX_WHEEL_SPEED = 20.0         # rad/s - must match RotationalMotor maxVelocity

ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
LEFT_WHEEL_JOINT = 'left_wheel_joint'
RIGHT_WHEEL_JOINT = 'right_wheel_joint'


class SecurityRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Motors (velocity control)
        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')
        for motor in (self.__left_motor, self.__right_motor):
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        # Wheel encoders
        self.__left_sensor = self.__robot.getDevice('left wheel sensor')
        self.__right_sensor = self.__robot.getDevice('right wheel sensor')
        self.__left_sensor.enable(self.__timestep)
        self.__right_sensor.enable(self.__timestep)

        # Odometry state
        self.__x = 0.0
        self.__y = 0.0
        self.__yaw = 0.0
        self.__prev_left = None
        self.__prev_right = None
        self.__prev_time = None

        self.__target_twist = Twist()

        # ROS interfaces
        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node('security_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__odom_pub = self.__node.create_publisher(Odometry, 'odom', 10)
        self.__joint_pub = self.__node.create_publisher(JointState, 'joint_states', 10)
        self.__tf_broadcaster = TransformBroadcaster(self.__node)

        self.__node.get_logger().info(
            'security_robot_driver ready: /cmd_vel in, /odom + /joint_states + TF odom->base_link out'
        )

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    @staticmethod
    def __clamp(value):
        return max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, value))

    def __apply_cmd_vel(self):
        v = self.__target_twist.linear.x
        w = self.__target_twist.angular.z

        # Differential-drive inverse kinematics
        left = (v - w * HALF_WHEEL_SEPARATION) / WHEEL_RADIUS
        right = (v + w * HALF_WHEEL_SEPARATION) / WHEEL_RADIUS

        self.__left_motor.setVelocity(self.__clamp(left))
        self.__right_motor.setVelocity(self.__clamp(right))

    def __update_odometry(self):
        left = self.__left_sensor.getValue()
        right = self.__right_sensor.getValue()
        if math.isnan(left) or math.isnan(right):
            return  # sensors not ready on the very first step

        sim_time = self.__robot.getTime()

        if self.__prev_left is None:
            self.__prev_left = left
            self.__prev_right = right
            self.__prev_time = sim_time
            return

        dt = sim_time - self.__prev_time
        if dt <= 0.0:
            return

        # Wheel travel since last step (m) and wheel angular velocity (rad/s)
        left_delta_rad = left - self.__prev_left
        right_delta_rad = right - self.__prev_right
        d_left = left_delta_rad * WHEEL_RADIUS
        d_right = right_delta_rad * WHEEL_RADIUS

        self.__prev_left = left
        self.__prev_right = right
        self.__prev_time = sim_time

        # Differential-drive forward kinematics (midpoint integration)
        d_center = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / (2.0 * HALF_WHEEL_SEPARATION)

        self.__x += d_center * math.cos(self.__yaw + d_yaw / 2.0)
        self.__y += d_center * math.sin(self.__yaw + d_yaw / 2.0)
        new_yaw = self.__yaw + d_yaw
        self.__yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))

        linear_velocity = d_center / dt
        angular_velocity = d_yaw / dt

        stamp = self.__node.get_clock().now().to_msg()
        qz = math.sin(self.__yaw / 2.0)
        qw = math.cos(self.__yaw / 2.0)

        # /odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = self.__x
        odom.pose.pose.position.y = self.__y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        pose_cov = [0.0] * 36
        pose_cov[0] = 0.01    # x
        pose_cov[7] = 0.01    # y
        pose_cov[14] = 1e6    # z (unused, planar robot)
        pose_cov[21] = 1e6    # roll (unused)
        pose_cov[28] = 1e6    # pitch (unused)
        pose_cov[35] = 0.03   # yaw
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.01
        twist_cov[7] = 1e6
        twist_cov[14] = 1e6
        twist_cov[21] = 1e6
        twist_cov[28] = 1e6
        twist_cov[35] = 0.03
        odom.twist.covariance = twist_cov

        self.__odom_pub.publish(odom)

        # TF odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = ODOM_FRAME
        tf.child_frame_id = BASE_FRAME
        tf.transform.translation.x = self.__x
        tf.transform.translation.y = self.__y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.__tf_broadcaster.sendTransform(tf)

        # /joint_states (wheel angles, so the wheels spin in RViz)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT]
        joints.position = [left, right]
        joints.velocity = [left_delta_rad / dt, right_delta_rad / dt]
        self.__joint_pub.publish(joints)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__apply_cmd_vel()
        self.__update_odometry()
