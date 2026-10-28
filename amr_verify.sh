#!/usr/bin/env bash

set -u

PASS=0
FAIL=0

check_result() {
    local name="$1"
    shift

    printf '\n[CHECK] %s\n' "$name"

    if "$@"; then
        printf '[PASS] %s\n' "$name"
        PASS=$((PASS + 1))
    else
        printf '[FAIL] %s\n' "$name"
        FAIL=$((FAIL + 1))
    fi
}

check_package() {
    ros2 pkg prefix "$1" >/dev/null 2>&1
}

check_topic_message() {
    local topic="$1"
    local timeout="${2:-5}"

    timeout "$timeout" ros2 topic echo "$topic" \
        --once >/dev/null 2>&1
}

check_transform() {
    timeout 5 ros2 run tf2_ros tf2_echo \
        odom base_link 2>&1 |
        grep -q 'Translation:'
}

check_controllers() {
    local output

    output="$(ros2 control list_controllers 2>/dev/null)" ||
        return 1

    printf '%s\n' "$output"

    printf '%s\n' "$output" |
        grep -Eq 'diff_drive_controller[[:space:]].*[[:space:]]active$' ||
        return 1

    printf '%s\n' "$output" |
        grep -Eq 'joint_state_broadcaster[[:space:]].*[[:space:]]active$'
}

printf '%s\n' '======================================'
printf '%s\n' ' AMR ROS 2 Runtime Verification'
printf '%s\n' '======================================'

if [[ -z "${ROS_DISTRO:-}" ]]; then
    printf '\n[FAIL] ROS environment is not sourced.\n'
    printf 'Run: source /opt/ros/jazzy/setup.bash\n'
    exit 2
fi

printf '\nROS_DISTRO=%s\n' "$ROS_DISTRO"

if [[ "$ROS_DISTRO" != "jazzy" ]]; then
    printf '[FAIL] Expected ROS 2 Jazzy.\n'
    exit 2
fi

check_result "amr_description package" check_package amr_description
check_result "amr_bringup package" check_package amr_bringup
check_result "amr_navigation package" check_package amr_navigation
check_result "amr_encoder_msgs package" check_package amr_encoder_msgs
check_result "amr_encoders package" check_package amr_encoders

check_result "Both controllers active" check_controllers

check_result "Wheel odometry publishes" \
    check_topic_message /diff_drive_controller/odom

check_result "EKF filtered odometry publishes" \
    check_topic_message /odometry/filtered

check_result "Front LiDAR publishes" \
    check_topic_message /front_lidar

check_result "Rear LiDAR publishes" \
    check_topic_message /rear_lidar

check_result "IMU publishes" \
    check_topic_message /imu

check_result "Left encoder publishes" \
    check_topic_message /left_encoder_ticks

check_result "Right encoder publishes" \
    check_topic_message /right_encoder_ticks

check_result "odom -> base_link transform available" \
    check_transform

printf '\n======================================\n'
printf 'SUMMARY: %d PASS, %d FAIL\n' "$PASS" "$FAIL"
printf '======================================\n'

if (( FAIL > 0 )); then
    exit 1
fi

exit 0
