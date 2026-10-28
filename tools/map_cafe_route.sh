#!/usr/bin/env bash
# Drive the service robot through the cafe for mapping (robot must start at 0,0 facing +x).
# Route: doorway -> kitchen north/south -> back -> north lane (y=+1) -> south lane (y=-1) -> start.
# Uses the EKF pose, no obstacle avoidance: the route keeps >= 0.5 m from furniture.
set -e
G="python3 $HOME/amr_ws/tools/goto.py"
for wp in "2.0 0.0 0.0" "4.3 0.0 0.0" "4.3 2.3 1.5708" "4.3 -2.3 -1.5708" "4.3 0.0 3.1416" \
          "2.0 0.0 3.1416" "0.0 1.0 3.1416" "-4.8 1.0 3.1416" "-4.8 -1.0 0.0" "0.0 -1.0 0.0" "0.0 0.0 0.0"; do
  echo "-> $wp"; $G $wp
done
echo ROUTE DONE
