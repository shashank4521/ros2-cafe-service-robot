#!/usr/bin/env bash
# Press "Done" (items loaded at the counter / order taken at the table).
ros2 topic pub --once -w 1 /delivery/confirm std_msgs/msg/Empty "{}" > /dev/null && echo "done sent"
