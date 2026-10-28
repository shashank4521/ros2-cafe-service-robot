"""Café delivery routine: counter -> table(s) -> charger, using the named places.

One trip:
    ros2 run amr_navigation delivery table_3 table_5
    ros2 run amr_navigation delivery table_2 --wait 20 --end counter

Serve mode (robot waits for orders, like in a real café):
    ros2 run amr_navigation delivery --serve
    ros2 topic pub --once /delivery/order std_msgs/msg/String "{data: 'table_3 table_5'}"

At the counter and at each table the robot waits until someone presses "Done"
(on the real robot: the touchscreen; here: this topic) or until --wait seconds pass:
    ros2 topic pub --once /delivery/confirm std_msgs/msg/Empty "{}"

Status (also shown above the robot in RViz):  ros2 topic echo /delivery/status
If a stop can't be reached, it clears the costmaps and tries once more, then skips it.
Places come from config/cafe_places.yaml.
"""

import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, Empty, String
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
ROLE_COLOUR = {'counter': (1.0, 0.6, 0.1), 'table': (0.2, 0.6, 1.0),
               'charger': (0.1, 0.8, 0.3), 'home': (0.8, 0.8, 0.8)}


def load_places(path=None):
    path = path or os.path.join(get_package_share_directory('amr_navigation'),
                                'config', 'cafe_places.yaml')
    raw = yaml.safe_load(open(path))
    return {name: dict(x=float(p['x']), y=float(p['y']), yaw=math.radians(float(p['yaw'])),
                       role=p.get('role', 'other')) for name, p in raw.items()}


class Delivery:

    def __init__(self, places, wait_s, end_place):
        self.places = places
        self.wait_s = wait_s
        self.end_place = end_place
        self.nav = BasicNavigator(node_name='delivery')
        self.nav.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.status_pub = self.nav.create_publisher(String, '/delivery/status', LATCHED)
        self.marker_pub = self.nav.create_publisher(MarkerArray, '/place_markers', LATCHED)
        self.status_marker_pub = self.nav.create_publisher(Marker, '/delivery/status_marker', 10)
        self.confirmed = False
        self.orders = []
        self.nav.create_subscription(Empty, '/delivery/confirm', self.on_confirm, 10)
        self.nav.create_subscription(String, '/delivery/order', self.on_order, 10)
        self.publish_place_markers()

    # ---------------- inputs ----------------
    def on_confirm(self, _msg):
        self.confirmed = True

    def on_order(self, msg):
        tables = msg.data.replace(',', ' ').split()
        bad = [t for t in tables if self.places.get(t, {}).get('role') != 'table']
        if not tables or bad:
            self.say(f'ORDER REJECTED: unknown table(s) {bad or tables}. '
                     f'Tables: {self.table_names()}')
            return
        self.orders.append(tables)
        self.say(f'ORDER RECEIVED: {" ".join(tables)} ({len(self.orders)} waiting)')

    def table_names(self):
        return [n for n, p in self.places.items() if p['role'] == 'table']

    # ---------------- outputs ----------------
    def say(self, text):
        self.nav.get_logger().info(text)
        self.status_pub.publish(String(data=text))
        m = Marker()                       # status sign floating above the robot
        m.header.frame_id = 'base_link'
        m.ns, m.id, m.type, m.action = 'delivery_status', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.x, m.pose.position.z = 0.2, 1.7
        m.pose.orientation.w = 1.0
        m.scale.z = 0.22
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.3, a=1.0)
        m.text = text.split(':')[0]
        self.status_marker_pub.publish(m)

    def publish_place_markers(self):
        arr = MarkerArray()
        for i, (name, p) in enumerate(self.places.items()):
            r, g, b = ROLE_COLOUR.get(p['role'], (1.0, 1.0, 1.0))
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.ns, arrow.id, arrow.type, arrow.action = 'place_arrows', i, Marker.ARROW, Marker.ADD
            arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = p['x'], p['y'], 0.05
            arrow.pose.orientation.z = math.sin(p['yaw'] / 2)
            arrow.pose.orientation.w = math.cos(p['yaw'] / 2)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.4, 0.07, 0.07
            arrow.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            label = Marker()
            label.header.frame_id = 'map'
            label.ns, label.id, label.type, label.action = 'place_names', i, Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = p['x'], p['y'], 0.5
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            label.text = name
            arr.markers += [arrow, label]
        self.marker_pub.publish(arr)

    # ---------------- actions ----------------
    def go(self, name):
        """Drive to a named place. Returns True if it arrived."""
        p = self.places[name]
        for attempt in (1, 2):
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.nav.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = p['x'], p['y']
            goal.pose.orientation.z = math.sin(p['yaw'] / 2)
            goal.pose.orientation.w = math.cos(p['yaw'] / 2)
            self.nav.goToPose(goal)
            while not self.nav.isTaskComplete():
                rclpy.spin_once(self.nav, timeout_sec=0.1)
            if self.nav.getResult() == TaskResult.SUCCEEDED:
                return True
            if attempt == 1:
                self.say(f'RETRYING: could not reach {name}, clearing costmaps and trying again')
                self.nav.clearAllCostmaps()
                time.sleep(1.0)
        return False

    def wait_for_done(self, what):
        """Wait at a stop until 'Done' is confirmed or the timeout passes."""
        self.confirmed = False
        t0 = time.time()
        while not self.confirmed and time.time() - t0 < self.wait_s:
            rclpy.spin_once(self.nav, timeout_sec=0.1)
        how = 'confirmed' if self.confirmed else f'timeout {self.wait_s:.0f} s'
        self.say(f'DONE at {what} ({how})')

    def trip(self, tables):
        report = []
        t_start = time.time()
        self.say(f'GOING TO COUNTER: to pick up for {" ".join(tables)}')
        if not self.go('counter'):
            self.say('FAILED: cannot reach the counter, trip cancelled')
            return [('counter', 'unreachable')]
        self.say(f'LOADING at counter: put the order for {" ".join(tables)} on the trays')
        self.wait_for_done('counter')
        for n, table in enumerate(tables, 1):
            self.say(f'DELIVERING ({n}/{len(tables)}): driving to {table}')
            t0 = time.time()
            if self.go(table):
                self.say(f'ARRIVED at {table}: please take your order')
                self.wait_for_done(table)
                report.append((table, f'delivered (drive {time.time() - t0:.0f} s)'))
            else:
                self.say(f'SKIPPED {table}: unreachable after 2 tries')
                report.append((table, 'SKIPPED - unreachable'))
        self.say(f'RETURNING to {self.end_place}')
        ok = self.go(self.end_place)
        report.append((self.end_place, 'arrived' if ok else 'unreachable'))
        self.say(f'TRIP FINISHED in {time.time() - t_start:.0f} s, at {self.end_place}'
                 if ok else f'TRIP FINISHED, but could not reach {self.end_place}')
        for stop, result in report:
            self.nav.get_logger().info(f'  {stop:10s} {result}')
        return report


def main():
    ap = argparse.ArgumentParser(description='Café delivery routine')
    ap.add_argument('tables', nargs='*', help='tables to deliver to, in order (one trip)')
    ap.add_argument('--serve', action='store_true', help='keep running and take orders on /delivery/order')
    ap.add_argument('--wait', type=float, default=10.0, help='seconds to wait at counter/table if nobody confirms')
    ap.add_argument('--end', default='charger', help='place to go after the last table (charger, counter, home)')
    ap.add_argument('--places', default=None, help='places yaml (default: config/cafe_places.yaml)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])

    rclpy.init()
    places = load_places(args.places)
    d = Delivery(places, args.wait, args.end)
    if args.end not in places:
        raise SystemExit(f'--end {args.end}: unknown place, choose from {list(places)}')
    if not args.serve and not args.tables:
        raise SystemExit(f'give tables (e.g. table_3 table_5) or --serve. Tables: {d.table_names()}')
    bad = [t for t in args.tables if places.get(t, {}).get('role') != 'table']
    if bad:
        raise SystemExit(f'unknown table(s) {bad}; tables: {d.table_names()}')

    d.say('WAITING FOR NAV2: starting up')
    d.nav.waitUntilNav2Active(localizer='amcl')
    try:
        if args.serve:
            d.say(f'READY: waiting for orders on /delivery/order (tables: {" ".join(d.table_names())})')
            while rclpy.ok():
                rclpy.spin_once(d.nav, timeout_sec=0.2)
                if d.orders:
                    d.trip(d.orders.pop(0))
                    d.say('READY: waiting for orders on /delivery/order')
        else:
            d.trip(args.tables)
    except KeyboardInterrupt:
        d.nav.cancelTask()
        d.say('STOPPED by user')
    finally:
        d.nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
