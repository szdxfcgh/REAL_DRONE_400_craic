#!/usr/bin/env bash
set -euo pipefail

warn() {
  printf 'WARN: %s\n' "$*" >&2
}

info() {
  printf 'INFO: %s\n' "$*"
}

if [[ ! -d "src" || ! -f "src/planner/craic_mission/package.xml" ]]; then
  printf 'ERROR: run this script from the catkin workspace root, for example:\n' >&2
  printf '  cd ~/craic_dev_for_onboard\n' >&2
  printf '  bash scripts/prepare_onboard_workspace.sh\n' >&2
  exit 1
fi

if ! command -v catkin_init_workspace >/dev/null 2>&1; then
  printf 'ERROR: catkin_init_workspace not found. Source ROS Noetic first:\n' >&2
  printf '  source /opt/ros/noetic/setup.bash\n' >&2
  exit 1
fi

if ! command -v catkin_make >/dev/null 2>&1; then
  printf 'ERROR: catkin_make not found. Source ROS Noetic first:\n' >&2
  printf '  source /opt/ros/noetic/setup.bash\n' >&2
  exit 1
fi

info "Repairing catkin workspace symlink"
if [[ -e "src/CMakeLists.txt" || -L "src/CMakeLists.txt" ]]; then
  rm -f "src/CMakeLists.txt"
fi
(cd src && catkin_init_workspace)

info "Marking sensor packages ignored for dry-run build"
ignore_dirs=(
  "src/realflight_modules/mid360_fastlio/src/livox_ros_driver/livox_ros_driver"
  "src/realflight_modules/mid360_fastlio/src/FAST_LIO"
  "src/realflight_modules/realsense-ros/realsense2_camera"
)

for dir in "${ignore_dirs[@]}"; do
  if [[ -d "$dir" ]]; then
    : > "$dir/CATKIN_IGNORE"
    info "Created $dir/CATKIN_IGNORE"
  else
    warn "Missing optional sensor package directory: $dir"
  fi
done

info "Cleaning previous catkin build outputs"
rm -rf build devel

info "Running catkin_make"
catkin_make

cat <<'EOF'

Workspace prepared.

Next steps:
  source devel/setup.bash
  roslaunch craic_mission competition_dryrun_full.launch
  rosrun craic_mission check_craic_topics.py
EOF
