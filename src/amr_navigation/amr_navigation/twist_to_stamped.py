"""Nav2 outputs geometry_msgs/Twist; diff_drive_controller (Jazzy) wants TwistStamped.

Subscribes:  in_topic  (Twist)         default /cmd_vel
Publishes:   out_topic (TwistStamped)  default /diff_drive_controller/cmd_vel
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistToStamped(Node):

    def __init__(self):
        super().__init__('twist_to_stamped')
        self.declare_parameter('in_topic', '/cmd_vel')
        self.declare_parameter('out_topic', '/diff_drive_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(TwistStamped, out_topic, 10)
        self.create_subscription(Twist, in_topic, self.on_twist, 10)
        self.get_logger().info(f'{in_topic} (Twist) -> {out_topic} (TwistStamped)')

    def on_twist(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistToStamped()
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
