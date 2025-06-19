"""ROS 2 node that converts wheel encoder ticks into an Odometry message and
(optional) TF transform for a mecanum base."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from math import sin, cos
from mecanum_drive.pose import Pose
from mecanum_drive import odometry
from builtin_interfaces.msg import Time

# Covariance matrices suggested by REP 105 for planar robots
ODOM_POSE_COVARIANCE = [
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
]

ODOM_TWIST_COVARIANCE = ODOM_POSE_COVARIANCE.copy()


class OdometryNode(Node):
    """Maintains robot pose by integrating wheel-encoder ticks."""

    def __init__(self):
        super().__init__('mecanum_drive_odometry')

        # Core odometry math lives in helper class
        self.odometry = odometry.Odometry()
        self.tf_broadcaster = TransformBroadcaster(self)

        # I/O topics
ODOM_POSE_COVARIANCE = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

ODOM_TWIST_COVARIANCE = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

class OdometryNode(Node):

    def __init__(self):
        super().__init__('mecanum_drive_odometry')
        self.odometry = odometry.Odometry()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odomPub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Int16MultiArray, 'wheel_ticks', self.ticksCallback, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_initial_pose, 10)

        # Parameters with defaults matching other nodes
        self.declare_parameter('ticks_per_meter', 4332.0)
        self.declare_parameter('wheel_separation', 0.34)
        self.declare_parameter('wheel_separation_length', 0.24)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('encoder_min', -32768)
        self.declare_parameter('encoder_max', 32767)
        self.declare_parameter('publish_tf', True)

        # Cache parameter values
        self.ticksPerMeter = self.get_parameter('ticks_per_meter').value
        self.wheelSeparation = self.get_parameter('wheel_separation').value
        self.wheelSeparationLength = self.get_parameter('wheel_separation_length').value
        self.rate_value = self.get_parameter('rate').value
        self.baseFrameID = self.get_parameter('base_frame_id').value
        self.odomFrameID = self.get_parameter('odom_frame_id').value
        self.encoderMin = self.get_parameter('encoder_min').value
        self.encoderMax = self.get_parameter('encoder_max').value
        self.publishTF = self.get_parameter('publish_tf').value

        # Feed geometry & calibration to helper
        self.odometry.setWheelSeparation(self.wheelSeparation)
        self.odometry.setWheelSeparationLength(self.wheelSeparationLength)
        self.odometry.setTicksPerMeter(self.ticksPerMeter)
        self.odometry.setEncoderRange(self.encoderMin, self.encoderMax)
        self.odometry.setTime(self.get_clock().now().nanoseconds * 1e-9)

        # Periodic pose publication
        self.timer = self.create_timer(1.0 / self.rate_value, self.publish)

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------

    def publish(self):
        """Integrate odometry and publish Odometry + optional TF."""
        self.timer = self.create_timer(1.0 / self.rate_value, self.publish)

    def publish(self):
        current_time = self.get_clock().now()
        self.odometry.updatePose(current_time.nanoseconds * 1e-9)
        pose = self.odometry.getPose()

        # Convert yaw to quaternion once for both TF and odom msg
        q = quaternion_from_euler(0, 0, pose.theta)

        # ---------------- TF ----------------
        q = quaternion_from_euler(0, 0, pose.theta)

        if self.publishTF:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odomFrameID
            t.child_frame_id = self.baseFrameID
            t.transform.translation.x = pose.x
            t.transform.translation.y = pose.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
            self.tf_broadcaster.sendTransform(t)

        # ---------------- Odometry message ----------------
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odomFrameID
        odom.child_frame_id = self.baseFrameID
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = pose.xVel
        odom.twist.twist.linear.y = pose.yVel
        odom.twist.twist.angular.z = pose.thetaVel
        odom.pose.covariance = ODOM_POSE_COVARIANCE
        odom.twist.covariance = ODOM_TWIST_COVARIANCE

        self.odomPub.publish(odom)

    def on_initial_pose(self, msg):
        """Reset pose when an RViz2 "2D Pose Estimate" is received."""
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(q)

        pose = Pose(x=msg.pose.pose.position.x,
                    y=msg.pose.pose.position.y,
                    theta=yaw)
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        pose = Pose()
        pose.x = msg.pose.pose.position.x
        pose.y = msg.pose.pose.position.y
        pose.theta = yaw

        self.get_logger().info(f'Setting initial pose to {pose}')
        self.odometry.setPose(pose)

    def ticksCallback(self, msg):
        """Forward raw wheel ticks to the odometry integrator."""
        self.odometry.updateWheels(msg.data[0], msg.data[1], msg.data[2], msg.data[3])


        self.odometry.updateWheels(msg.data[0], msg.data[1], msg.data[2], msg.data[3])

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
