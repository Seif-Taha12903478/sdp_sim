#!/usr/bin/env bash
# Installs the SDP simulation files into the Hiwonder robot_gazebo package and rebuilds it.
# Usage:  bash ~/sdp_sim/install.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PKG="$HOME/ros2_ws/src/simulations/robot_gazebo"
[ -d "$PKG" ] || { echo "robot_gazebo not found at $PKG"; exit 1; }

# hospital system (branch hospital-system): new file names only, main's files are not affected
for f in urdf/jetarm_mecanum.gazebo.xacro worlds/hospital.sdf worlds/hospital_video.sdf worlds/hospital_photo.sdf launch/hospital.launch.py \
         urdf/jetarm_mecanum_system.gazebo.xacro worlds/hospital_system.sdf worlds/hospital_system_video.sdf \
         worlds/hospital_system_photo.sdf launch/hospital_system.launch.py; do
  if [ -f "$PKG/$f" ] && ! cmp -s "$HERE/robot_gazebo/$f" "$PKG/$f"; then
    cp "$PKG/$f" "$PKG/$f.bak.$(date +%Y%m%d%H%M%S)"
  fi
  cp "$HERE/robot_gazebo/$f" "$PKG/$f"
  echo "installed $f"
done

echo "--- checking Gazebo plugins"
PLUG=$(dirname "$(find /usr -name 'libignition-gazebo-detachable-joint-system.so' 2>/dev/null | head -1)")
for p in velocity-control odometry-publisher sensors detachable-joint contact joint-position-controller joint-state-publisher; do
  if ls "$PLUG"/libignition-gazebo-$p-system.so >/dev/null 2>&1; then echo "  ok   $p"; else echo "  MISSING $p"; fi
done
python3 -c "import cv2, numpy; print('  ok   python3 cv2', cv2.__version__)" || echo "  MISSING python3-opencv (sudo apt install python3-opencv)"

echo "--- building robot_gazebo"
cd "$HOME/ros2_ws" && colcon build --packages-select robot_gazebo
echo
echo "Done. Open a NEW terminal, then:  ros2 launch robot_gazebo hospital.launch.py"
echo "Hospital system:                   ros2 launch robot_gazebo hospital_system.launch.py gui:=false"
