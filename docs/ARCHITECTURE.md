# Architecture

How the parts of the café robot connect: packages, nodes, topics, TF frames, and the
order things start in.

## Packages

| Package | Type | Role |
|---|---|---|
| `amr_description` | ament_cmake | Robot models: `service_robot.urdf.xacro` (café cart), `amr.urdf.xacro` (warehouse AMR); `ros2_control` controller YAMLs |
| `amr_bringup` | ament_python | `sim.launch.py` (starts everything), worlds (`cafe.sdf`, `amr_warehouse.sdf`), bridge + EKF configs, RViz layouts, `tof_scan_to_range`, `dashboard`, café world generator |
| `amr_navigation` | ament_python | Maps, Nav2 / SLAM / AMCL params, `cafe_places.yaml`, `delivery`, `twist_to_stamped`, `map_to_markers`, param and no-go-zone generators |
| `amr_encoders` | ament_python | `wheel_encoder_node`: wheel angles → encoder ticks |
| `amr_encoder_msgs` | ament_cmake | Custom message `WheelEncoderTicks` (`int64 ticks`, `uint32 cpr`) |

## Our own nodes

| Node | Listens to | Publishes | Why it exists |
|---|---|---|---|
| `wheel_encoder_node` | `/joint_states` | `/left_encoder_ticks`, `/right_encoder_ticks` | Gives the same data a real encoder would |
| `tof_scan_to_range` (×4) | `/tof_*` (tiny LaserScan from Gazebo) | `/tof_*/range` (`Range`) | Real ToF sensors give one distance, not a scan. Reports `max_range` when clear |
| `twist_to_stamped` | `/cmd_vel` (`Twist`) | `/diff_drive_controller/cmd_vel` (`TwistStamped`) | Nav2 outputs `Twist`; the Jazzy diff drive controller wants `TwistStamped` |
| `map_to_markers` | map / zones / costmap | `/zone_markers`, `/obstacle_markers` | Clear coloured RViz overlays |
| `delivery` | `/delivery/order`, `/delivery/confirm` | `/delivery/status`, markers; calls the `NavigateToPose` action | The café job: counter → tables → charger |
| `dashboard` | ~15 topics | (window / PNG) | Shows what every sensor and algorithm is doing |

## Main third-party nodes

| Node | Package | Job |
|---|---|---|
| Gazebo + `ros_gz_bridge` | ros_gz | Fake world; copies sensor topics and `/clock` into ROS |
| `robot_state_publisher` | robot_state_publisher | URDF → fixed TF frames |
| `controller_manager`, `diff_drive_controller`, `joint_state_broadcaster` | ros2_control | Drive the wheels, report wheel angles and odometry |
| `ekf_filter_node` | robot_localization | Wheel speeds + IMU yaw rate → `odom → base_link` |
| `slam_toolbox` | slam_toolbox | Builds the map (mapping mode only) |
| `map_server`, `amcl` | nav2 | Serve the map; localize the robot → `map → odom` |
| `planner_server`, `controller_server`, `bt_navigator`, `behavior_server`, `velocity_smoother`, `collision_monitor` | nav2 | Plan, follow, recover, smooth, keep safe |
| second `map_server` (`/keepout_zones`) | nav2 | No-go zones (behind the counter, around tables) |

## TF tree

```
map ──(AMCL)──> odom ──(EKF)──> base_link ──(robot_state_publisher)──> lidar_link
                                                                  ├──> camera_link
                                                                  ├──> imu_link
                                                                  ├──> left/right wheel, front swivel wheels
                                                                  └──> tof_front_left/right, tof_rear_left/right
```

`base_link` sits in the middle of the rear (driven) axle, because that's the point the
robot turns around.

Rule learned the hard way: **each TF link must have exactly one publisher.** The diff drive
controller is told not to publish `odom → base_link` (`enable_odom_tf: false`) so it doesn't
fight with the EKF.

## Costmaps

Both the global and local costmaps use:
- **Static layer:** the SLAM map (walls, partition, counter, pillar).
- **3D voxel layer:** LiDAR + depth camera (the local one adds the 4 ToF sensors). It's 3D so
  the high LiDAR seeing "nothing" at 1.25 m doesn't erase a box the camera saw at floor level.
- **Keepout filter:** no-go zones from `maps/cafe_zones.pgm`.
- **Inflation:** 0.70 m soft "danger glow".

Footprint: 0.64 × 0.54 m around the rear axle (real body + 5 cm padding).

## Start-up order (`sim.launch.py`)

1. Gazebo with the world
2. `robot_state_publisher` (URDF from xacro)
3. Spawn the robot
4. Bridge (sensors, `/clock`)
5. Controllers (joint state broadcaster, diff drive)
6. EKF, encoders, ToF converters (and the LiDAR merger for the AMR)
7. After ~8 s: SLAM **or** Nav2 (+ AMCL, zones map server, `twist_to_stamped`)
8. Optional: RViz, dashboard

## Generated files

Some configs are written by scripts so they can be changed in one place and regenerated:

| Script | Writes |
|---|---|
| `src/amr_bringup/tools/gen_cafe_world.py` | `src/amr_bringup/worlds/cafe.sdf` |
| `src/amr_navigation/tools/gen_cafe_zones.py` | `src/amr_navigation/maps/cafe_zones.*` |
| `src/amr_navigation/tools/gen_nav2_params_service.py` | `src/amr_navigation/config/nav2_params_service.yaml` |
| `src/amr_navigation/tools/gen_nav2_params.py` | `src/amr_navigation/config/nav2_params.yaml` (AMR) |
