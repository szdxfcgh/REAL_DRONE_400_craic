# Coding Rules

Scope rules:
- Mission logic belongs in `src/planner/craic_mission`.
- Keep PX4_Ctrl, Ego-Planner core, plan_env, FAST-LIO, Livox, and Realsense unchanged unless the task explicitly authorizes those files.
- Do not refactor unrelated packages while adding mission behavior.

ROS rules:
- Put configurable topics, frames, waypoints, limits, and timeouts in YAML.
- Use existing ROS message types when possible.
- Keep topic types stable and document them in the final report.
- Use `rospy.Time.now()` for runtime watchdogs and publish state transitions for test visibility.
- On odom loss, mission timeout, or geofence violation, enter a landing path.

Testing rules:
- Run syntax checks for Python nodes before launch tests.
- Run `catkin_make` from the workspace root in a sourced ROS environment.
- Verify topics with `rostopic info`, `rostopic echo`, and `rqt_graph`.
- Test with fake data before hardware.
- Do not proceed to props-on testing until dry-run and props-off checks pass.
