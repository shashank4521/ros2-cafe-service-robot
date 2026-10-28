"""Draw an OccupancyGrid as RViz markers (workaround for RViz's Map display rendering white).

On this laptop RViz's Map display fails with a GLSL error
("active samplers with a different type refer to the same texture image unit"),
so grids show as a blank white rectangle. Markers use a different shader.

Parameters
  grid_topic    input OccupancyGrid            (default /map)
  marker_topic  output MarkerArray             (default /map_markers)
  style         map     : walls black + free space light grey  (latched topics like /map)
                zones   : occupied cells orange                 (no-go zones)
                costmap : lethal/inscribed cells red            (live costmaps)
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

STYLES = {
    # style: list of (namespace, test on cell value, rgb, alpha, z)
    'map': [('free', lambda v: (v >= 0) & (v < 25), (0.85, 0.85, 0.85), 1.0, -0.01),
            ('occupied', lambda v: v >= 65, (0.0, 0.0, 0.0), 1.0, 0.0)],
    'zones': [('zones', lambda v: v >= 65, (1.0, 0.55, 0.1), 0.6, 0.005)],
    'costmap': [('obstacles', lambda v: v >= 99, (0.9, 0.1, 0.1), 0.8, 0.02)],
}


class GridToMarkers(Node):

    def __init__(self):
        super().__init__('map_to_markers')
        grid_topic = self.declare_parameter('grid_topic', '/map').value
        marker_topic = self.declare_parameter('marker_topic', '/map_markers').value
        self.style = self.declare_parameter('style', 'map').value
        if self.style not in STYLES:
            raise ValueError(f'style must be one of {list(STYLES)}')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        # Live costmaps are re-published often: a volatile subscriber accepts any publisher.
        sub_qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
                   if self.style == 'costmap' else latched)
        self.pub = self.create_publisher(MarkerArray, marker_topic, latched)
        self.create_subscription(OccupancyGrid, grid_topic, self.on_grid, sub_qos)
        self.logged = False

    def layer(self, msg, ns, mid, cells, rgb, alpha, z):
        info = msg.info
        m = Marker()
        m.header = msg.header
        m.ns, m.id, m.type, m.action = ns, mid, Marker.CUBE_LIST, Marker.ADD
        m.pose = info.origin
        m.scale.x = m.scale.y = info.resolution
        m.scale.z = 0.01
        m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=alpha)
        rows, cols = np.divmod(cells, info.width)
        m.points = [Point(x=float((c + 0.5) * info.resolution), y=float((r + 0.5) * info.resolution),
                          z=float(z)) for r, c in zip(rows, cols)]
        return m

    def on_grid(self, msg):
        data = np.asarray(msg.data, dtype=np.int16)
        markers = []
        for i, (ns, test, rgb, alpha, z) in enumerate(STYLES[self.style]):
            markers.append(self.layer(msg, ns, i, np.nonzero(test(data))[0], rgb, alpha, z))
        self.pub.publish(MarkerArray(markers=markers))
        if not self.logged or self.style != 'costmap':
            counts = ', '.join(f'{m.ns}: {len(m.points)}' for m in markers)
            self.get_logger().info(f'{msg.info.width}x{msg.info.height} grid -> {counts}')
            self.logged = True


def main():
    rclpy.init()
    node = GridToMarkers()
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
