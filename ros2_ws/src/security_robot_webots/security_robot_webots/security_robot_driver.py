import rclpy
from geometry_msgs.msg import Twist

WHEEL_RADIUS = 0.03            # m  - must match the wheel Cylinder radius in the world file
HALF_WHEEL_SEPARATION = 0.09   # m  - wheel y-offset in the world file
MAX_WHEEL_SPEED = 20.0         # rad/s - must match RotationalMotor maxVelocity


class SecurityRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        for motor in (self.__left_motor, self.__right_motor):
            motor.setPosition(float('inf'))   # velocity control mode
            motor.setVelocity(0.0)

        self.__target_twist = Twist()

        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node('security_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.get_logger().info('security_robot_driver ready, listening on /cmd_vel')

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    @staticmethod
    def __clamp(value):
        return max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, value))

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        v = self.__target_twist.linear.x
        w = self.__target_twist.angular.z

        # Differential-drive inverse kinematics
        left = (v - w * HALF_WHEEL_SEPARATION) / WHEEL_RADIUS
        right = (v + w * HALF_WHEEL_SEPARATION) / WHEEL_RADIUS

        self.__left_motor.setVelocity(self.__clamp(left))
        self.__right_motor.setVelocity(self.__clamp(right))
