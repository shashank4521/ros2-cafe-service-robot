#!/usr/bin/env bash
# Stop everything started by sim.launch.py (and stragglers)
pkill -INT -f 'ros2 launch amr_bringup [s]im.launch.py'
for i in $(seq 30); do pgrep -f '[g]z sim|/opt/ros/jazzy/lib/(nav2_|opennav|robot_localization|controller_manager|ros_gz)' >/dev/null || break; sleep 0.5; done
for p in $(pgrep -f '[g]z sim|/opt/ros/jazzy/lib/(nav2_|opennav|robot_localization|controller_manager|ros_gz|robot_state_publisher|rclcpp_components|slam_toolbox)|[w]heel_encoder_node|[t]of_scan_to_range|[t]wist_to_stamped'); do kill $p; done
pkill -INT -x rviz2; pkill -f '[m]ap_to_markers'
sleep 2
# Gazebo's window sometimes ignores a normal stop: force it
for p in $(pgrep -f '[g]z sim|/opt/ros/jazzy/lib/(nav2_|opennav)'); do kill -9 $p; done
pkill -f '[m]ap_to_markers|[t]wist_to_stamped|[w]heel_encoder_node|[t]of_scan_to_range|amr_bringup/[d]ashboard|amr_navigation/[d]elivery'
for p in $(pgrep -x rviz2) $(pgrep -f 'amr_bringup/[d]ashboard'); do kill -9 $p; done
sleep 1
if pgrep -f '[g]z sim|/opt/ros/jazzy/lib/|amr_ws/install/' >/dev/null; then
  pgrep -af '[g]z sim|/opt/ros/jazzy/lib/|amr_ws/install/'
else
  # Nothing left running: clear Fast DDS shared-memory leftovers from killed processes.
  # Stale ones stop new ROS programs from discovering each other.
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
  echo all-stopped
fi
