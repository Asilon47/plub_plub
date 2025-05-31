import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

from mecanum_drive import controller

class ControllerNode(Node):

    def __init__(self):
        super().__init__('mecanum_drive_controller')

        self.controller = controller.Controller()
        self.linearXVelocity = 0.0
        self.linearYVelocity = 0.0
        self.angularVelocity = 0.0
        self.wheels_to_send = Int16MultiArray()

        self.declare_parameter('ticks_per_meter', 13606.30)
        self.declare_parameter('wheel_separation', 0.34)
        self.declare_parameter('wheel_separation_length', 0.24)
        self.declare_parameter('max_motor_speed', (3456))
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('timeout', 0.2)

        self.ticksPerMeter = self.get_parameter('ticks_per_meter').value
        self.wheelSeparation = self.get_parameter('wheel_separation').value
        self.wheelSeparationLength = self.get_parameter('wheel_separation_length').value
        self.maxMotorSpeed = self.get_parameter('max_motor_speed').value
        self.rate_value = self.get_parameter('rate').value
        self.timeout = self.get_parameter('timeout').value

        self.controller.setWheelSeparation(self.wheelSeparation)
        self.controller.setWheelSeparationLength(self.wheelSeparationLength)
        self.controller.setTicksPerMeter(self.ticksPerMeter)
        self.controller.setMaxMotorSpeed(self.maxMotorSpeed)

        self.wheelPub = self.create_publisher(Int16MultiArray, 'wheels_desired_rate', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.twistCallback, 10)

        self.lastTwistTime = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_value, self.publish)

    def publish(self):
        now = self.get_clock().now()
        time_diff = (now - self.lastTwistTime).nanoseconds * 1e-9
        if time_diff < self.timeout:
            speeds = self.controller.getSpeeds(self.linearXVelocity, self.linearYVelocity, self.angularVelocity)
            self.wheels_to_send.data = [int(speeds.frontLeft), int(speeds.frontRight), int(speeds.rearLeft), int(speeds.rearRight)]
        else:
            self.wheels_to_send.data = [0, 0, 0, 0]
        self.wheelPub.publish(self.wheels_to_send)

    def twistCallback(self, twist):
        self.linearXVelocity = twist.linear.x
        self.linearYVelocity = twist.linear.y
        self.angularVelocity = twist.angular.z
        self.lastTwistTime = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
