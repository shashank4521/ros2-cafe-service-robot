#!/usr/bin/env bash
# Send an order to the robot running "ros2 run amr_navigation delivery --serve"
#   order.sh table_3 table_5
# -w 1 waits until the robot is listening, so the order is never lost.
[ $# -eq 0 ] && { echo "usage: order.sh table_N [table_M ...]"; exit 1; }
ros2 topic pub --once -w 1 /delivery/order std_msgs/msg/String "{data: '$*'}" > /dev/null && echo "order sent: $*"
