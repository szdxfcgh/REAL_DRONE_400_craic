# Project Rules

This is a ROS Noetic UAV workspace based on REAL_DRONE_400, Ego-Planner, PX4_Ctrl, FAST-LIO, Livox, and Realsense integrations.

Hard rules:
- Do not modify PX4_Ctrl low-level controller unless explicitly requested.
- Do not modify Ego-Planner core code unless explicitly requested.
- Do not modify FAST-LIO, Livox, or Realsense drivers unless explicitly requested.
- Prefer adding mission-level code under `src/planner/craic_mission`.
- Keep changes small, reviewable, and testable.
- Every code change must include build and test commands.
- Do not remove existing topics, params, message definitions, or launch file names used by existing scripts.
- Safety-critical code must fail safe by stopping the mission and requesting land.
- Do not run real-flight code without a verified baseline, props-off checks, RC takeover, and operator approval.
