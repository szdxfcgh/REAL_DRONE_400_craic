# CRAIC Mission First Version

This package is the first mission-level implementation for the CRAIC 2026 micro UAV task.

## Scope

The first version keeps the existing FAST-LIO, EGO Planner and px4ctrl stack, and adds a mission FSM that publishes dynamic goals to `/move_base_simple/goal`.

It currently treats K230 vision and the physical drop mechanism as replaceable ROS interfaces:

- QR result: `/craic/qr_result`, `std_msgs/String`, format `class_a,class_b,left` or `class_a,class_b,right`.
- Target detection: `/craic/target_detected`, `std_msgs/String`, format `class,confidence,center_x,center_y`.
- Ring center override: `/craic/ring_pose`, `geometry_msgs/PoseStamped`.
- Drop command: `/craic/drop_cmd`, `std_msgs/Int32`, values `1`, `2`, `3`.

## K230 Serial

The preferred K230 integration is the in-package serial node `k230_serial_node.py`.
It expects one serial line per recognition result:

```text
ROS_MSG:qr,man,apple,left
ROS_MSG:target,man,0.87,320,240
ROS_MSG:target,apple,0.82,300,250
ROS_MSG:ring,7.50,0.00,1.60
```

The copied K230-side reference script is `scripts/k230_device_main.py`. It is meant to run on the K230/CanMV side and print the protocol lines above to the serial console.

Serial mode:

```bash
roslaunch craic_mission k230_serial_node.launch serial_port:=/dev/ttyACM0 baud_rate:=115200
```

Full dry-run stack with K230 serial enabled:

```bash
roslaunch craic_mission competition_all.launch dry_run:=true auto_start:=false start_k230_serial:=true k230_serial_port:=/dev/ttyACM0
```

`k230_bridge.py` is still kept for UDP tests and accepts legacy `QR:...`, `TARGET:...`, and `RING:...` lines, but the serial node above matches the current K230 `ROS_MSG:...` protocol directly.

## Safe Dry Run

Build first:

```bash
cd /home/reallab/real_drone_400
catkin_make
source devel/setup.bash
```

Start the stack without sending takeoff or landing commands:

```bash
roslaunch craic_mission competition_all.launch dry_run:=true auto_start:=false
```

With `dry_run:=true`, `competition_all.launch` starts `fake_odom.py` by default. It publishes `/Odom_high_freq` and follows every `/move_base_simple/goal`, so the mission FSM can complete without the aircraft.

Trigger the mission manually:

```bash
rostopic pub -1 /craic_mission_fsm/start std_msgs/Empty "{}"
```

Useful dry-run checks:

```bash
rostopic echo /move_base_simple/goal
rostopic echo /craic/drop_cmd
rostopic echo /craic_mission_fsm/state
```

Right-side landing mock:

```bash
roslaunch craic_mission competition_all.launch dry_run:=true auto_start:=false mock_qr_text:=man,apple,right
```

## Flight Run

Only use this after props-off bench tests and RC failsafe checks:

```bash
roslaunch craic_mission competition_all.launch dry_run:=false auto_start:=false
```

Then trigger with the same `/craic_mission_fsm/start` command.

## Parameters To Measure Before Real Flight

Edit `config/field.yaml` after measuring the actual local frame:

- `points/obstacle_center`
- `points/image_target_a`
- `points/image_target_b`
- `points/special_target`
- `points/ring_pre`, `points/ring_center`, `points/ring_post`
- `points/landing_left`, `points/landing_right`

The default frame assumes the takeoff point is `(0, 0, 0)`, +x follows the 9 m field direction, and +y points to the left side when looking along +x.
