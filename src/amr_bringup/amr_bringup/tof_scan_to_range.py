import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class ToFScanToRange(Node):

    def __init__(self):
        super().__init__('tof_scan_to_range')

        self.tof_subscriptions = []
        self.tof_publishers = {}

        tof_names = [
            'tof_front_left',
            'tof_front_right',
            'tof_rear_left',
            'tof_rear_right',
        ]

        for tof_name in tof_names:
            self.tof_subscriptions.append(
                self.create_subscription(
                    LaserScan,
                    f'/{tof_name}',
                    lambda msg, name=tof_name: self.scan_callback(msg, name),
                    10,
                )
            )

            self.tof_publishers[tof_name] = self.create_publisher(
                Range,
                f'/{tof_name}/range',
                10,
            )

    def scan_callback(self, msg, tof_name):
        valid_ranges = [
            value for value in msg.ranges
            if msg.range_min <= value <= msg.range_max
        ]

        # A real ToF reports the nearest object in its cone.
        # Nothing in range -> max_range (like many ToF drivers). Not +inf: Nav2's
        # collision monitor treats an out-of-range reading as a broken sensor and
        # stops the robot.
        distance = min(valid_ranges) if valid_ranges else msg.range_max

        output = Range()

        output.header = msg.header
        output.radiation_type = Range.INFRARED
        output.field_of_view = msg.angle_max - msg.angle_min
        output.min_range = msg.range_min
        output.max_range = msg.range_max
        output.range = distance

        self.tof_publishers[tof_name].publish(output)


def main(args=None):
    rclpy.init(args=args)

    node = ToFScanToRange()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
