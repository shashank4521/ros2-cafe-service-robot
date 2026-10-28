#!/usr/bin/env bash
# Follow what the delivery robot is doing (Ctrl+C to stop watching).
ros2 topic echo /delivery/status std_msgs/msg/String --qos-durability transient_local --qos-reliability reliable --field data
