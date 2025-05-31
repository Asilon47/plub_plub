#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray
import struct
import can


class CANNode(Node):
    """
    A ROS 2 node that:
      - Subscribes to a topic (`wheels_desired_rate`) to receive 4×Int16 rates.
      - Buffers the latest rates and then sends them over CAN once every 0.1 s.
      - Listens for incoming CAN frames on a given ID and publishes parsed wheel
        ticks as an Int16MultiArray on `wheel_ticks`.
    """

    def __init__(self):
        super().__init__('can_node')
        self.get_logger().info("Initializing CANNode...")

        # Declare & read ROS parameters (with defaults)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('tx_can_id', 0x00000010)
        self.declare_parameter('rx_can_id', 0x00000001)
        self.declare_parameter('filter_incoming', True)

        self.can_interface = self.get_parameter('can_interface').value
        self.tx_can_id = self.get_parameter('tx_can_id').value
        self.rx_can_id = self.get_parameter('rx_can_id').value
        self.filter_incoming = self.get_parameter('filter_incoming').value

        # Subscription: just update the buffer of desired wheel rates
        self.last_rates = [0, 0, 0, 0]
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'wheels_desired_rate',
            self.desired_rate_callback,
            10
        )

        # Publisher for parsed wheel ticks from incoming CAN
        self.wheel_ticks_pub = self.create_publisher(
            Int16MultiArray,
            'wheel_ticks',
            10
        )

        # Initialize CAN bus
        self.get_logger().info(f"Setting up CAN bus on '{self.can_interface}'...")
        try:
            self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
        except Exception as e:
            self.get_logger().error(f"Error setting up CAN interface: {e}")
            raise

        # Listener + Notifier for incoming CAN messages
        self.listener = CANListener(
            wheel_ticks_pub=self.wheel_ticks_pub,
            rx_can_id=self.rx_can_id,
            filter_incoming=self.filter_incoming,
            logger=self.get_logger()
        )
        self.notifier = can.Notifier(self.bus, [self.listener], timeout=0.01)

        # Timer: send buffered rates every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_send_callback)

        self.get_logger().info("CANNode initialized successfully.")

    def desired_rate_callback(self, msg: Int16MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(f"Expected 4 wheel rates, got {len(msg.data)}")
            return
        # Buffer the latest rates; sending happens in timer
        self.last_rates = list(msg.data)

    def timer_send_callback(self):
        # Pack four signed 16-bit ints into 8 bytes (big-endian)
        payload = struct.pack(
            '>hhhh',
            self.last_rates[0],
            self.last_rates[1],
            self.last_rates[2],
            self.last_rates[3],
        )

        can_msg = can.Message(
            arbitration_id=self.tx_can_id,
            data=payload,
            is_extended_id=True
        )

        try:
            self.bus.send(can_msg)
            hex_str = ' '.join(f"{b:02X}" for b in payload)
            self.get_logger().info(f"Sent CAN data on ID {hex(self.tx_can_id)}: {hex_str}")
        except can.CanError as e:
            self.get_logger().error(f"Failed to send CAN message: {e}")


class CANListener(can.Listener):
    """
    A python-can Listener that parses 4×Int16 wheel positions from incoming CAN frames
    and publishes them as Int16MultiArray on the 'wheel_ticks' topic.
    """
    def __init__(self, wheel_ticks_pub, rx_can_id, filter_incoming, logger):
        super().__init__()
        self.wheel_ticks_pub = wheel_ticks_pub
        self.rx_can_id = rx_can_id
        self.filter_incoming = filter_incoming
        self.logger = logger

    def on_message_received(self, msg: can.Message):
        # Filter by ID if requested
        if self.filter_incoming and msg.arbitration_id != self.rx_can_id:
            return

        if len(msg.data) < 8:
            self.logger.warn("Received CAN message with fewer than 8 bytes—ignoring.")
            return

        # Parse four signed 16-bit ints (big-endian)
        data = msg.data
        fl = int.from_bytes(data[0:2], byteorder='big', signed=True)
        fr = int.from_bytes(data[2:4], byteorder='big', signed=True)
        rl = int.from_bytes(data[4:6], byteorder='big', signed=True)
        rr = int.from_bytes(data[6:8], byteorder='big', signed=True)

        self.logger.info(
            f"Received CAN msg on ID {hex(msg.arbitration_id)}: "
            f"fl={fl}, fr={fr}, rl={rl}, rr={rr}"
        )

        out = Int16MultiArray()
        out.data = [fl, fr, rl, rr]
        self.wheel_ticks_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CANNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down CANNode.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
