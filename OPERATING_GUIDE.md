# AMR Simulation — Operating Guide

Everything below runs from a terminal on this laptop. Each new terminal needs:

```bash
source /opt/ros/jazzy/setup.bash
source ~/amr_ws/install/setup.bash
```

Tip: add both lines to the end of `~/.bashrc` so every terminal is ready.

---

**Autonomous service robot in the café: see `CAFE_NAV_GUIDE.md` (start here).**

## 0. Which robot?

| Robot | Launch option | Status |
|---|---|---|
| Service cart (0.6 x 0.52 m, 360° LiDAR on mast, depth camera, IMU, encoders, 4 ToF) | `robot:=service` | everything incl. Nav2 in the café (`world:=cafe nav:=true`) |
| Old AMR (1.5 x 1.0 m, 2 LiDARs) | `robot:=amr` (default) | everything incl. Nav2 |

Service robot, Gazebo + RViz:
```bash
ros2 launch amr_bringup sim.launch.py robot:=service rviz:=true
```
RViz (service.rviz) shows: LiDAR scan (red), depth camera points (coloured by height),
camera image panel, the 4 ToF cones (yellow), robot model. Fixed frame is `odom`.

Drive the service robot to a point (uses the EKF pose; robot starts at 0,0):
```bash
python3 ~/amr_ws/tools/goto.py 1.5 1.0 0.0     # x y yaw  (next to the box)
```
Check all its sensors against the room: `python3 ~/amr_ws/tools/sensor_check_service.py`
(robot must be at the start position).

Keyboard driving works the same (section 3); the service robot goes up to 0.5 m/s.

**Café world** (`world:=cafe`; other worlds: `amr_warehouse`, or a full .sdf path):
```bash
ros2 launch amr_bringup sim.launch.py robot:=service world:=cafe rviz:=true
```
Spawn somewhere else with `x:=… y:=… yaw:=…` (default 0, 0, 0 = room centre).

Re-map the café (robot must start at 0,0):
```bash
ros2 launch amr_bringup sim.launch.py robot:=service world:=cafe slam:=true rviz:=true
~/amr_ws/tools/map_cafe_route.sh                  # ~4 min automatic route
python3 ~/amr_ws/tools/cafe_map_check.py /tmp/cafe.png   # checks every feature
ros2 run nav2_map_server map_saver_cli -f ~/amr_ws/src/amr_navigation/maps/cafe
```
If the saver prints no `Map saved`, just run it again (the first call sometimes misses the map).

Change the café (move tables, add furniture): edit the LAYOUT block in
`src/amr_bringup/tools/gen_cafe_world.py`, run it with `python3`, then
`colcon build --symlink-install --packages-select amr_bringup`.

---

## 1. Start the simulation

| What you want | Command |
|---|---|
| Robot in Gazebo, sensors, EKF (drive manually) | `ros2 launch amr_bringup sim.launch.py` |
| …and build a new map | `ros2 launch amr_bringup sim.launch.py slam:=true rviz:=true` |
| …and navigate on the saved map | `ros2 launch amr_bringup sim.launch.py nav:=true rviz:=true` |
| No Gazebo window (faster, less CPU) | add `gui:=false` |
| Skip the 4 ToF sensors | add `tof:=false` |
| Use a different map | add `map:=/full/path/to/map.yaml` |

Wait ~15 s after launch. Nav2 is ready when the log shows
`lifecycle_manager_navigation: Managed nodes are active`.

**Stop:** press `Ctrl+C` in the launch terminal. If anything is left running
(e.g. Gazebo did not close), run `~/amr_ws/tools/stop_sim.sh`.

Only run one simulation at a time — two Gazebo instances will fight over topics.

---

## 2. Check that everything is healthy

With the simulation running, in a second terminal:

```bash
bash ~/amr_ws/amr_verify.sh
```

All 14 lines should say `[PASS]` (AMR only: it checks the front/rear LiDARs, so the
service robot fails those two; use `tools/sensor_check_service.py` for it). Useful one-off checks:

```bash
ros2 topic hz /merged               # ~10 Hz merged LiDAR
ros2 topic hz /odometry/filtered    # EKF output
ros2 control list_controllers       # both controllers "active"
ros2 run tf2_ros tf2_echo map base_link   # robot pose on the map (needs slam or nav)
```

---

## 3. Drive the robot by hand (keyboard)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_link -r cmd_vel:=/diff_drive_controller/cmd_vel
```

Keys: `i` forward, `,` back, `j`/`l` turn, `k` stop. The robot is limited to 0.1 m/s
and 0.1 rad/s by `controllers.yaml`, whatever speed the keyboard asks for.
Don't drive by hand while Nav2 is driving.

---

## 4. Build and save a new map

1. `ros2 launch amr_bringup sim.launch.py slam:=true rviz:=true`
2. Drive slowly around the area (keyboard, section 3), or run the automatic square:
   `python3 ~/amr_ws/tools/drive_square.py` (2 x 2 m square, ~3 minutes).
   Make sure every wall has been seen; grey areas in RViz are unknown.
3. Save (this overwrites the current map):
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/amr_ws/src/amr_navigation/maps/warehouse
   ```
4. Optional quality check while SLAM is still running:
   `python3 ~/amr_ws/tools/map_check.py /tmp/map.png` (prints occupied/unknown %, writes an image).
5. Rebuild so the new map is installed:
   ```bash
   cd ~/amr_ws && colcon build --symlink-install --packages-select amr_navigation
   ```

---

## 5. Navigate

Start with `nav:=true rviz:=true` (section 1). The robot starts at the map origin and
AMCL already knows that, so no initial pose is needed after a fresh launch.

**In RViz:**
- **2D Pose Estimate** (toolbar): click-drag where the robot really is, if AMCL is lost
  (scan points in red don't line up with the black map walls).
- **2D Goal Pose**: click-drag the destination and direction. The blue line is the plan,
  the green box is the robot footprint.

**From the command line** (one goal):
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: -1.5}, orientation: {w: 1.0}}}}"
```

**Scripted test** (3 goals, compares with Gazebo's true pose):
```bash
python3 ~/amr_ws/src/amr_navigation/tools/send_test_goals.py
```
Expected: 3x `SUCCEEDED`, goal error < 0.25 m, AMCL error < 0.1 m. Takes ~3 minutes.

Coordinates: map x = along the room's 12 m length (−6…+6), y = across (−4…+4).
The box is at (3, 1.5). Keep goals at least ~1 m from walls (robot is 1.5 x 1.0 m).

---

## 6. Measure odometry accuracy

With the sim running (no Nav2, so nothing else is driving):
```bash
python3 ~/amr_ws/tools/measure_odometry.py spin -0.1 15.7   # ~90° turn in place
python3 ~/amr_ws/tools/measure_odometry.py line 0.1 20      # ~2 m straight
python3 ~/amr_ws/tools/measure_odometry.py spin 0.0 120     # 2 min standing still (drift)
```
It prints Gazebo truth vs wheel joints, wheel odometry, IMU and EKF.
Good result today: all within ~0.5% of truth; standstill drift < 0.01 rad.

---

## 7. Where to change things

| To change… | Edit | Then |
|---|---|---|
| Robot shape, wheels, sensor positions | `src/amr_description/urdf/amr.urdf.xacro` | restart sim |
| Speed / acceleration limits, wheel size | `src/amr_description/config/controllers.yaml` | restart sim |
| EKF (what it fuses) | `src/amr_bringup/config/ekf.yaml` | restart sim |
| LiDAR merger | `src/amr_bringup/config/dual_laser_merger.yaml` (offsets stay 0!) | restart sim |
| SLAM settings | `src/amr_navigation/config/mapper_params_online_async.yaml` | restart sim |
| Nav2 / AMCL settings | `src/amr_navigation/config/nav2_params.yaml` | restart sim |
| World | `src/amr_bringup/worlds/amr_warehouse.sdf` | restart sim |

Most files are symlinked into `install/` so edits apply on restart. **New** files
(new launch file, new map, new world) need a rebuild:
```bash
cd ~/amr_ws && colcon build --symlink-install
```

Rules that bit us before:
- The merger reads LiDAR positions from the URDF; its `*_offset` values are extra
  corrections only. Keep them 0.
- Only the EKF may publish `odom → base_link`: keep `enable_odom_tf: false`.
- Use **sphere** collisions for wheels and casters; cylinders give wrong turning in Gazebo.
- The world inside `amr_warehouse.sdf` is named `empty`. Use `-world empty` for spawn.
- Change one thing at a time and re-measure (sections 2, 5, 6).

---

## 8. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Robot doesn't move with keyboard | Forgot `-p stamped:=true` or the remap; check `ros2 control list_controllers` |
| Robot doesn't move with Nav2 | `ros2 topic echo /cmd_vel` (Nav2 output) and `/diff_drive_controller/cmd_vel` (after adapter) |
| RViz: "Fixed frame [map] does not exist" | No SLAM/Nav2 running → set Fixed Frame to `odom` |
| RViz map is a plain white rectangle, log shows `GLSL link result` | RViz's Map display shader fails on this laptop's Mesa driver. Use the **Map (markers)** display (on by default, fed by `map_to_markers`, which `sim.launch.py` starts). The Map / costmap displays are switched off in amr.rviz for this reason |
| RViz shows nothing from /merged | LaserScan display must use Reliability **Best Effort** (preset in amr.rviz) |
| Map smeared / doubled walls | Run section 6; check merger offsets are 0 |
| AMCL lost (scan doesn't match map) | Set **2D Pose Estimate** in RViz |
| Goal fails "progress checker" | Robot blocked or turning too long; see `movement_time_allowance` in nav2_params.yaml |
| colcon error `[Errno 17] File exists` | An old plain copy in `install/` blocks a symlink: delete that one file in `install/…` and rebuild |
| Leftover processes / topics doubled | `~/amr_ws/tools/stop_sim.sh` |
