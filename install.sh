#!/usr/bin/env bash
# Installs the TrackMed simulation files into the Hiwonder robot_gazebo package and rebuilds it.
# Usage:  bash ~/sdp_sim/install.sh
# Needs:  ROS 2 Humble + Gazebo Fortress, the Hiwonder packages in ~/ros2_ws/src/simulations
#         (see README.md, "Installation"), the repo cloned to ~/sdp_sim (the launch files start
#         the helper scripts from there).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SIM="$HOME/ros2_ws/src/simulations"
PKG="$SIM/robot_gazebo"
for p in jetarm_6dof_description robot_gazebo; do
  [ -d "$SIM/$p" ] || { echo "Hiwonder package $p not found in $SIM (see README.md, Installation)"; exit 1; }
done
[ "$HERE" = "$HOME/sdp_sim" ] || echo "WARNING: the repo is at $HERE; the launch files expect $HOME/sdp_sim"
[ -n "$CAMERA_TYPE" ] || echo "WARNING: CAMERA_TYPE is not set (the xacro needs it): echo 'export CAMERA_TYPE=GEMINI' >> ~/.bashrc"

# The generated worlds reference textures and container models by ABSOLUTE path. They are
# committed for /home/ubuntu/sdp_sim; anywhere else they are regenerated (deterministic).
if ! grep -q "$HERE/robot_gazebo/worlds/" "$HERE/robot_gazebo/worlds/hospital.sdf"; then
  echo "--- regenerating the worlds for $HERE"
  python3 "$HERE/tools/make_hospital_world.py" > /dev/null
  python3 "$HERE/tools/make_system_world.py" > /dev/null
fi

# new file names only: the Hiwonder files are not changed (an older copy is kept as .bak.DATE)
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

echo "--- building jetarm_6dof_description robot_gazebo"
cd "$HOME/ros2_ws" && colcon build --packages-select jetarm_6dof_description robot_gazebo
echo
echo "Done. Open a NEW terminal, then:  ros2 launch robot_gazebo hospital.launch.py"
echo "Hospital system:                   ros2 launch robot_gazebo hospital_system.launch.py gui:=false"
