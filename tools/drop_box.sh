#!/usr/bin/env bash
# Drop a box obstacle into the running cafe sim:  drop_box.sh X Y [SIZE] [HEIGHT] [NAME]
# Remove it again:                                  drop_box.sh remove [NAME]
W=cafe
if [ "$1" = remove ]; then
  N=${2:-dropped_box}
  gz service -s /world/$W/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
    --timeout 3000 --req "name: \"$N\" type: MODEL"; exit
fi
X=$1; Y=$2; S=${3:-0.5}; H=${4:-0.6}; N=${5:-dropped_box}
SDF="<sdf version='1.10'><model name='$N'><static>true</static><pose>$X $Y $(python3 -c "print($H/2)") 0 0 0</pose><link name='l'><collision name='c'><geometry><box><size>$S $S $H</size></box></geometry></collision><visual name='v'><geometry><box><size>$S $S $H</size></box></geometry><material><ambient>0.9 0.2 0.2 1</ambient><diffuse>0.9 0.2 0.2 1</diffuse></material></visual></link></model></sdf>"
gz service -s /world/$W/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 3000 --req "sdf: \"$SDF\""
