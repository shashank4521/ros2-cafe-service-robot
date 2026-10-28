# Service Robot in the Café — Navigation Guide

How to run the autonomous service robot yourself: start it, send it to places,
drop obstacles in its way, and change its behaviour.

Every new terminal first needs:
```bash
source /opt/ros/jazzy/setup.bash
source ~/amr_ws/install/setup.bash
```
(Put both lines at the end of `~/.bashrc` and you never have to type them again.)

---

## 1. Start everything (one command)

```bash
ros2 launch amr_bringup sim.launch.py robot:=service world:=cafe nav:=true rviz:=true
```

Wait about 20 seconds. It's ready when the terminal shows
`lifecycle_manager_navigation: Managed nodes are active`.

Two windows open:
- **Gazebo**: the 3D café with the robot (the real "world").
- **RViz**: what the robot *thinks*: its map, position, sensors and plans.

The robot starts at the room centre (0, 0) facing the kitchen, and AMCL already
knows that, so you can send goals straight away.

Faster, no 3D window: add `gui:=false` (RViz still works).

**Stop:** `Ctrl+C` in that terminal. If anything is left over (Gazebo window stays,
or the next start can't see topics), run:
```bash
~/amr_ws/tools/stop_sim.sh
```

---

## 2. What you see in RViz

| Colour / item | Meaning |
|---|---|
| Black lines + light grey floor | The saved café map (walls, partition, pillar, counter display, fridge, shelf, plants) |
| **Orange rectangles** | No-go zones: tables + chairs, counter base, charging dock. The mast LiDAR can't see these (too low), so they are drawn in by hand |
| **Red cells** | Obstacles the robot sees *right now* (LiDAR, depth camera, ToF) |
| Red dots | 360° LiDAR scan (1.25 m high, sees walls and tall things only) |
| Coloured points | Depth camera point cloud (sees tables, chairs, boxes, people) |
| Yellow cones | The 4 ToF sensors (short range, near the floor) |
| Camera panel | What the RealSense-style camera sees |
| Pink arrows | AMCL's guesses of where the robot is. Tight cluster = confident |
| Blue line | The planned path |
| Green box | The robot's outline (with its 5 cm safety margin) |

---

## 3. Send the robot somewhere

### a) Click in RViz
1. Click **2D Goal Pose** in the top toolbar.
2. Click on the map where the robot should go, **hold and drag** to set the
   direction it should face, then release.
3. It plans (blue line) and drives there, going around obstacles.

Pick spots in open floor, at least ~0.7 m from walls, tables and the counter
(the robot is 0.6 x 0.52 m and turns around its *rear* axle, so its front swings out).

### b) Named places (like a real café robot)
```bash
python3 ~/amr_ws/src/amr_navigation/tools/cafe_goals.py counter table_3 home
```
Places: `counter`, `table_1` … `table_6`, `charger`, `home`.
With no names it runs a 7-stop tour. For each stop it prints whether it succeeded,
how close it got, the localization error and the closest it came to furniture.

| Place | Where (x, y) | Faces |
|---|---|---|
| counter | (4.60, 0.00) | the counter (east) |
| table_1 / table_2 / table_3 | (−4.3 / −1.5 / 1.3, **+1.05**) | north, towards the table |
| table_4 / table_5 / table_6 | (−4.3 / −1.5 / 1.3, **−1.05**) | south, towards the table |
| charger | (4.50, −2.75) | the dock (south) |
| home | (0, 0) | the kitchen (east) |

### c) One goal from the command line
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.6, y: 0.0}, orientation: {w: 1.0}}}}"
```

To cancel: send a new goal, or press `Ctrl+C` in the script's terminal.

---

## 4. Deliveries (the café routine)

The robot does what a real café robot does:
**counter (load) → table → (next table…) → charger**, waiting at each stop until
someone presses "Done" (on the real robot: the touchscreen) or a timeout passes.

### a) One trip
```bash
ros2 run amr_navigation delivery table_3 table_5
```
Options: `--wait 20` (seconds to wait at each stop, default 10), `--end counter`
(where to go afterwards, default `charger`).

### b) Serve mode (robot waits for orders all day)
Terminal 1, the robot:
```bash
ros2 run amr_navigation delivery --serve --wait 60
```
Terminal 2, you (the café staff):
```bash
~/amr_ws/tools/order.sh table_2 table_6     # new order (several tables = one trip)
~/amr_ws/tools/done.sh                      # "Done": loaded at counter / taken at table
~/amr_ws/tools/status.sh                    # watch what it is doing (Ctrl+C to stop)
```
Orders sent while it is busy are queued. Unknown tables are rejected with a message.

### What it says (status)
| Status | Meaning |
|---|---|
| `READY` | Waiting for an order |
| `GOING TO COUNTER` | Driving to pick up |
| `LOADING at counter` | Put the items on the trays, then `done.sh` |
| `DELIVERING (1/2)` | Driving to a table |
| `ARRIVED at table_N` | Customer takes the order, then `done.sh` |
| `RETRYING` | Couldn't reach a stop: clears its obstacle memory and tries once more |
| `SKIPPED table_N` | Still unreachable: skips it, keeps delivering the rest, reports it at the end |
| `TRIP FINISHED` | Back at the charger, with a per-table report in the robot's terminal |

The same status floats above the robot in RViz (yellow text), and every named place
is shown with its name and an arrow for the direction the robot will face
(orange = counter, blue = tables, green = charger).

### Change the places
All stops are in `src/amr_navigation/config/cafe_places.yaml` (x, y in metres,
yaw in degrees, role). Add e.g. `table_7: {x: 0.0, y: -1.05, yaw: -90, role: table}`,
test it once with `cafe_goals.py table_7`, then it works in orders. The file is
linked into `install/`, so no rebuild is needed after editing; restart `delivery`.

Tested 24 Oct: one trip (table_3, table_5) ✓; serve mode with confirmations
(table_2, table_6) ✓; bad order "table_9" rejected ✓; table_4 blocked by a box →
retried, skipped, table_1 still delivered, back to charger ✓.

---

## 5. Live dashboard: what the robot measures and decides

Start it together with everything else:
```bash
ros2 launch amr_bringup sim.launch.py robot:=service world:=cafe nav:=true rviz:=true dashboard:=true
```
or on its own while the sim runs:
```bash
ros2 run amr_bringup dashboard
ros2 run amr_bringup dashboard --ros-args -p save_png:=$HOME/dash.png   # also save a picture every 5 s
```

One window, updated 5× per second, last 30 seconds in the time plots:

| Panel | What it shows | What to look for |
|---|---|---|
| **Turn rate** | wheel odometry (orange), IMU gyro (cyan), EKF (white) | All three on top of each other = healthy. Orange lower/higher than cyan = wheels slipping or mis-calibrated; white should stay with cyan |
| **Speed** | Nav2 controller command (purple), after the collision monitor (red), actual (white) | Red dropping below purple = the safety layer slowed or stopped the robot |
| **Heading** | wheel-only (orange), EKF (white), AMCL on the map (green dashed) | Orange drifting away = wheel odometry error building up; white/green together = fused heading is good |
| **Decisions & state** | controller command, safety action, delivery status, AMCL pose ± σ, how much AMCL corrects the EKF, nearest obstacle per sensor | The "why" behind what the robot is doing right now |
| **LiDAR** | 360° scan, top view, robot in the middle facing right | Walls only (it's 1.25 m high): tables are invisible here |
| **Depth / Colour camera** | what the RealSense-style camera sees (depth in metres, colours = distance) | Tables, chairs, boxes appear here even though the LiDAR misses them |
| **ToF** | 4 low short-range sensors (2.0 m = nothing seen) | Orange < 1 m, red < 0.3 m |
| **Path** | EKF-only track (orange) vs AMCL-corrected track (green dashed) | They separate when the EKF drifts; AMCL pulls the robot back onto the map |
| **Localization** | map→odom shift in cm (how much AMCL is correcting), AMCL σ (uncertainty) | A slowly growing shift = normal EKF drift being fixed; σ jumping up = AMCL unsure (lost?) |
| **Wheel encoders** | left / right wheel speed | Same = straight, different = turning, opposite = turning on the spot |

"EKF heading σ" grows over time: the EKF has no compass, so it can only become less
sure of its heading. That's normal, and exactly why AMCL is there.

### See the EKF save the day (wheel-slip demo)
```bash
ros2 launch amr_bringup sim.launch.py robot:=service world:=cafe nav:=true rviz:=true dashboard:=true wheel_fault:=1.15
```
`wheel_fault:=1.15` makes the wheel odometry under-report every turn by 13 %, like
slipping or badly calibrated wheels. Send a goal or two and watch:
- **Turn rate:** orange peaks are lower than cyan; white follows cyan (EKF trusts the IMU).
- **Heading:** orange drifts tens of degrees away; white and green stay together.
- The robot still reaches its goals (tested: 6–8 cm, AMCL 2–6 cm).
Example picture: `~/amr_ws/dashboard_wheel_fault_example.png`.

Optional: for drag-and-drop plots of any topic (and replaying recordings), install
PlotJuggler: `sudo apt install ros-jazzy-plotjuggler-ros`, then `ros2 run plotjuggler plotjuggler`.

---

## 6. Test obstacle avoidance

Drop a red box (0.5 m wide, 0.6 m tall: lower than the LiDAR, so only the camera
and ToF can see it) anywhere in the running café:
```bash
~/amr_ws/tools/drop_box.sh 1.6 0.0          # x y  [size] [height] [name]
~/amr_ws/tools/drop_box.sh -3.0 -0.3 0.4 0.8 box2
~/amr_ws/tools/drop_box.sh remove           # remove "dropped_box"
~/amr_ws/tools/drop_box.sh remove box2
```
Then send a goal on the other side of it. Watch the red cells appear in RViz when
the camera sees it, and the blue path bend around it.

To include a box in the clearance report:
```bash
python3 ~/amr_ws/src/amr_navigation/tools/cafe_goals.py counter home box=1.6,0,0.5
```

---

## 7. If something goes wrong

| What you see | What to do |
|---|---|
| Pink arrows spread out / LiDAR dots don't line up with black walls | Robot is "lost": use **2D Pose Estimate** (toolbar) and click-drag where it really is |
| Robot stops and waits near an obstacle | Normal: it is too close to turn safely. Send it a goal in open space, or remove the obstacle |
| `done.sh` seems ignored | It was sent before the robot arrived (it only listens while waiting at a stop). Send it again |
| Goal "FAILED" | Goal was inside/too close to an orange zone or wall, or the way is blocked. Pick a more open spot |
| Dashboard window won't close | Close it with the window's X, or `~/amr_ws/tools/stop_sim.sh` |
| Nothing moves, no errors | Check `ros2 topic hz /scan` and `ros2 control list_controllers` (both controllers "active") |
| `ros2 topic list` shows almost nothing although the sim runs | Leftovers from a crashed run: `~/amr_ws/tools/stop_sim.sh`, then start again |
| Map is a white rectangle in RViz | Graphics bug on this laptop; use the "Map (markers)" display (on by default) |

---

## 8. How it works (one paragraph per part)

1. **Sensors → EKF:** wheel encoders + IMU are fused by the EKF into smooth motion
   (`odom → base_link`). It drifts slowly.
2. **AMCL:** compares the live LiDAR scan with the saved map 10× per second and
   corrects the drift (`map → odom`). Typical error here: 1–5 cm.
3. **Costmaps:** a big *global* one (map + no-go zones + seen obstacles) for planning,
   and a 5 x 5 m *local* one around the robot (LiDAR + depth camera + 4 ToF) for driving.
   Both are **3D** (voxels), so the high LiDAR can't erase low obstacles the camera saw.
4. **Planner (NavFn):** finds the cheapest path, staying in the middle of lanes.
5. **Controller (Regulated Pure Pursuit):** follows the path, slows near obstacles and
   in curves, turns on the spot when needed, and checks the robot's real outline
   (+5 cm) for collisions before every move.
6. **Collision monitor:** last safety check on every speed command: slows to 40% when
   something is within ~0.4 m in front, and stops just before a predicted contact.
7. **Adapter:** Nav2 outputs `Twist` on `/cmd_vel`; `twist_to_stamped` converts it to
   the `TwistStamped` the wheel controller needs.

---

## 9. Change things

| To change… | Edit | Then |
|---|---|---|
| Named places (add "table_7", move stops) | `src/amr_navigation/config/cafe_places.yaml` | restart `delivery` |
| Speed, turn rate, safety margin, goal accuracy | `ROBOT` block in `src/amr_navigation/tools/gen_nav2_params_service.py` | `python3` that file, restart the sim |
| Café layout (tables, counter…) | `LAYOUT` in `src/amr_bringup/tools/gen_cafe_world.py` | run it, re-map (OPERATING_GUIDE §0), run `gen_cafe_zones.py`, rebuild |
| No-go zones | `src/amr_navigation/tools/gen_cafe_zones.py` | run it, restart the sim |
| Robot body / sensors | `src/amr_description/urdf/service_robot.urdf.xacro` | restart the sim |

Don't edit `config/nav2_params_service.yaml` by hand: it is generated.
After adding *new* files, rebuild: `cd ~/amr_ws && colcon build --symlink-install`.

---

## 10. What we learned getting here (useful on the real robot)

- **A mast LiDAR can't see tables or chairs.** The fix: no-go zones on the map +
  depth camera in the costmaps.
- **Use 3D (voxel) costmaps when sensors are at different heights.** In 2D, the high
  LiDAR beams pass over a low box and erase it, and the planner drives into it.
- **Rear-axle robots swing their front out when turning.** Stops must leave room
  for the swing (tables: 1.05 m from the table centre; counter: 4.60 m).
- **The camera only sees what's in front.** Sides are blind for low objects, so the
  outline gets a 5 cm safety margin (10 cm was too much: normal stops became "collisions").
- **ToF "nothing seen" must be reported as max range, not infinity.** Nav2's collision
  monitor treats infinity as a broken sensor and stops the robot.
- **Simple beats clever here:** NavFn + Pure Pursuit beat Smac Hybrid-A* and MPPI for this
  café (7/7 tours) once the costmaps were right.

Test results (24 Oct): 7/7 tour without obstacles, 7/7 with a box in the aisle,
3/3 with the box moved: arrival 4–13 cm, AMCL 0.3–5 cm, never closer than 17 cm
to any furniture, wall or box.
