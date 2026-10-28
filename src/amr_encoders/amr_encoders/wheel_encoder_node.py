import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from amr_encoder_msgs.msg import WheelEncoderTicks


class WheelEncoderNode(Node):

    CPR = 1024

    def __init__(self):
        super().__init__('wheel_encoder_node')

        self.left_publisher = self.create_publisher(
            WheelEncoderTicks,
            '/left_encoder_ticks',
            10,
        )

        self.right_publisher = self.create_publisher(
            WheelEncoderTicks,
            '/right_encoder_ticks',
            10,
        )

        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        self.get_logger().info(
            'Wheel encoder node started: 1024 CPR'
        )

    def joint_state_callback(self, msg):
        try:
            left_index = msg.name.index('left_wheel_joint')
            right_index = msg.name.index('right_wheel_joint')
        except ValueError:
            return

        if left_index >= len(msg.position) or right_index >= len(msg.position):
            return

        left_ticks = self.position_to_ticks(msg.position[left_index])
        right_ticks = self.position_to_ticks(msg.position[right_index])

        left_msg = WheelEncoderTicks()
        left_msg.ticks = left_ticks
        left_msg.cpr = self.CPR

        right_msg = WheelEncoderTicks()
        right_msg.ticks = right_ticks
        right_msg.cpr = self.CPR

        self.left_publisher.publish(left_msg)
        self.right_publisher.publish(right_msg)

    @staticmethod
    def position_to_ticks(position):
        return int(round(position / (2.0 * math.pi) * WheelEncoderNode.CPR))


def main(args=None):
    rclpy.init(args=args)

    node = WheelEncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
