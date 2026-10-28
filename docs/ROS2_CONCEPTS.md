# ROS 2 Concepts, Explained With This Project

Every idea is explained simply first, then tied to a real file in this repo and a command
to try. Start the simulation first:

```bash
ros2 launch amr_bringup sim.launch.py robot:=service world:=cafe nav:=true rviz:=true
```

---

## 1. Node — one cook, one job
A **node** is a small program that does one job. ROS is like a restaurant kitchen: many cooks
(nodes), each doing one thing, talking to each other.

- In this repo: `src/amr_encoders/amr_encoders/wheel_encoder_node.py` hears wheel angles and says encoder ticks.
- Try: `ros2 node list` then `ros2 node info /wheel_encoder_node`

## 2. Package and workspace — shelves and the building
A **package** is a folder of code and settings for one purpose. The **workspace** (`amr_ws`)
holds all packages. `colcon build` turns `src/` into `install/`.

- In this repo: 5 packages in `src/` (see [ARCHITECTURE.md](ARCHITECTURE.md)).

## 3. Topic — a radio channel
A **topic** is a named channel. **Publishers** talk, **subscribers** listen, and neither knows
about the other. Each topic carries one **message type**.

- In this repo: the LiDAR talks on `/scan`; AMCL and the costmaps listen.
- `twist_to_stamped.py` is a one-subscriber, one-publisher node: it turns `Twist` into `TwistStamped`.
- Try: `ros2 topic list`, `ros2 topic hz /scan`, `ros2 topic echo /imu --once`, `rqt_graph`

## 4. Service — a phone call
A **service** is a question with one quick answer.

- In this repo: `delivery.py` calls the "clear costmap" services when a table is blocked.
- Try: `ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"`

## 5. Action — a job that takes time
An **action** is a long job with a goal, progress updates (feedback), a result, and a cancel button.

- In this repo: "drive to table 3" is a `NavigateToPose` action. RViz's 2D Goal Pose and `delivery.py` both send it.
- Try: `ros2 action list`, then `ros2 interface show nav2_msgs/action/NavigateToPose`

| Use a… | When |
|---|---|
| Topic | Data that flows all the time (sensors, speed commands) |
| Service | A quick question/command ("clear the costmap") |
| Action | A long task with progress ("drive to table 3") |

## 6. Interface — the form
Messages (`.msg`), services (`.srv`) and actions (`.action`) are **forms with fixed boxes**.

- In this repo: our own form `src/amr_encoder_msgs/msg/WheelEncoderTicks.msg` (`ticks`, `cpr`).
- Try: `ros2 interface show sensor_msgs/msg/LaserScan`

## 7. Parameter — a knob
A **parameter** is a setting you change without editing code. A **YAML file** is a list of knob settings given to a node at start.

- In this repo: `src/amr_bringup/config/ekf.yaml`, `src/amr_navigation/config/nav2_params_service.yaml`
- Try: `ros2 param get /controller_server FollowPath.desired_linear_vel`, then `ros2 param set ... 0.2`

## 8. Launch file — the opening checklist
A **launch file** starts many nodes, hands each its YAML, and connects (remaps) topics. One command opens the whole kitchen.

- In this repo: `src/amr_bringup/launch/sim.launch.py`, with switches `robot:=`, `world:=`, `slam:=`, `nav:=`, `rviz:=`
- Try: `ros2 launch amr_bringup sim.launch.py --show-args`

## 9. URDF / xacro — the body drawing
**URDF** describes the robot's parts (**links**) and how they connect (**joints**). Each link has
a look (`visual`), a bumping shape (`collision`) and a weight (`inertial`). **Xacro** adds
shortcuts so repeated parts are written once.

- In this repo: `src/amr_description/urdf/service_robot.urdf.xacro`
- Lesson: wheel collisions are **spheres**. Cylinders touched the floor on their inner rim and made odometry wrong by 11 %.

## 10. TF — the family tree of positions
Every part has its own point of view (**frame**). **TF** links them, so a chair the LiDAR sees
2 m ahead can be placed on the café map.

`map → odom → base_link → sensors`
- `base_link → sensors`: fixed, from the URDF (`robot_state_publisher`)
- `odom → base_link`: how far I've moved (EKF)
- `map → odom`: correction so I match the map (AMCL)
- Try: `ros2 run tf2_tools view_frames`, `ros2 run tf2_ros tf2_echo map base_link`

## 11. Gazebo and the bridge — the video-game world and its translator
**Gazebo** simulates physics and fake sensors. The **bridge** copies Gazebo topics into ROS.
The rest of ROS can't tell fake from real, so on the real robot only the drivers change.

- In this repo: `src/amr_bringup/worlds/cafe.sdf` (made by `tools/gen_cafe_world.py`), `src/amr_bringup/config/service_bridge.yaml`
- Try: `gz topic -l`

## 12. ros2_control — the motor driver
`diff_drive_controller` turns "go 0.3 m/s, turn 0.5 rad/s" into left/right wheel speeds, and
wheel angles back into odometry.

- In this repo: `src/amr_description/config/service_controllers.yaml`

## 13. EKF — "how am I moving?"
Wheels slip; the IMU slowly drifts. The **Extended Kalman Filter** mixes both and trusts each
where it's good. We feed it **speeds** (not positions) from the wheels and **turn rate** from the IMU.

- In this repo: `src/amr_bringup/config/ekf.yaml`
- Demo: `wheel_fault:=1.15` makes the wheels lie by 13 %; the EKF + IMU keep the robot on track.

## 14. SLAM — drawing the map
**SLAM** (Simultaneous Localization And Mapping) is like walking a dark room with a torch,
drawing the map as you go and using the drawing to know where you are. Done once, then saved.

- In this repo: `slam:=true`, `tools/map_cafe_route.sh`, result `src/amr_navigation/maps/cafe.pgm`

## 15. AMCL — "where am I on the map?"
**AMCL** spreads hundreds of guesses (particles). Guesses whose imagined LiDAR view matches the
real scan survive; the rest die. The survivors cluster where the robot really is.

- Try: in RViz, set a wrong pose with **2D Pose Estimate**, drive, and watch the particles regroup.

## 16. Nav2 — the sat-nav and the driver
| Part | Kid version |
|---|---|
| Costmap | The map with "danger paint" around walls, tables, boxes and no-go zones |
| Planner (NavFn) | Draws the route line |
| Controller (Regulated Pure Pursuit) | Chases a carrot a little ahead on the line; slows near obstacles |
| Velocity smoother | No jerky starts, so drinks don't spill |
| Collision monitor | Last-second brake |
| Behavior tree | The boss's flowchart: plan → follow → if stuck, clear and retry |

## 17. Putting it together: one goal click
1. A goal is sent → `NavigateToPose` **action**.
2. The planner draws a path on the costmap (map + LiDAR + depth camera + zones).
3. The controller sends `/cmd_vel` 20× per second (**topic**).
4. Smoother → collision monitor → `twist_to_stamped` → wheel controller → the wheels turn.
5. Wheels + IMU → EKF → `odom → base_link` (**TF**). LiDAR + map → AMCL → `map → odom`.
6. The controller sees the robot got closer, and the loop repeats until it arrives.

**Sense → think → move → sense again.**
