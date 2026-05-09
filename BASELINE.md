# Baseline

Date: 2026-05-09
Workspace: `D:/暑/ROS/机载电脑文件/REAL_DRONE_400/REAL_DRONE_400`
Branch observed: `main`

## Current Verification Status

This file records the baseline that must be checked before mission-code changes are used on hardware.

Checked in the current Codex Windows shell:
- Python AST syntax check is available.
- `catkin_make` is not available in the current PowerShell PATH.
- ROS runtime commands were not executed in this shell.

Required baseline checks in the ROS/Linux environment:

```bash
cd /home/reallab/real_drone_400
catkin_make
source devel/setup.bash
roslaunch ego_planner single_run_in_exp.launch
roslaunch px4ctrl run_ctrl.launch
```

Record results here before flight:
- `catkin_make`:
- Ego-Planner launch:
- PX4Ctrl launch:
- `/Odom_high_freq` rate:
- `/move_base_simple/goal` subscriber:
- `/px4ctrl/takeoff_land` subscriber:
- RC takeover:
- Emergency stop / disarm:

## Safety Gate

Do not run new mission code on a real vehicle until this baseline is filled in and a git checkpoint exists.
