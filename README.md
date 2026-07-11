# MECA500 3D Printing
A ROS2 package for controlling a Meca500 6DOF robot arm (5 μm resolution) to perform robotic 3D printing via MoveIt2.

## Overview
This project bridges the Meca500 proprietary API to MoveIt2 through a custom ROS2 hardware interface, enabling real trajectory planning and execution on physical hardware. A print pipeline sweeps the robot's reachable workspace, centers and clips sliced G-code onto the densest reachable region without resizing the model, and executes it move-by-move through MoveIt2's Pilz Industrial Motion Planner (LIN for straight/extruding moves, CIRC for arcs).

## Packages

* meca500_hardware — ROS2 hardware interface bridging the Meca500 API to MoveIt2
* meca500_moveit — MoveIt2 configuration and launch files
* meca500_robot — Robot description (URDF/Xacro), including the mounted extruder end-effector and `nozzle` tool frame
* meca500_demo — Reachability sweeping, planning scene setup, and the main print-execution node: parses G-code moves, plans them through Pilz LIN/CIRC, and recovers from occasional IK failures (Z-hop for travels, midpoint bisection for extruding moves) without skipping a commanded point
* gcode — G-code handling: a Python preprocessing tool (centers a sliced print on the densest reachable region, drops moves outside the workspace as gaps instead of resizing, validates and repairs arc geometry) plus a C++ parser library used by the trajectory node at execution time

## Demos

* **With extruder, flat bed (Benchy)** — full print pipeline end to end
  _[video placeholder]_
* **No extruder, random-orientation bed (cube)** — table tilted to an arbitrary pose via `/table_service`
  _[video placeholder]_
* **G2/G3 (arc) moves**
  _[video placeholder]_
* **G1 (straight-line) moves**
  https://private-user-images.githubusercontent.com/172546714/600223649-309005d1-acc4-417c-8442-79d06b82b89f.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODAwNzI4MzksIm5iZiI6MTc4MDA3MjUzOSwicGF0aCI6Ii8xNzI1NDY3MTQvNjAwMjIzNjQ5LTMwOTAwNWQxLWFjYzQtNDE3Yy04NDQyLTc5ZDA2YjgyYjg5Zi5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNTI5JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDUyOVQxNjM1MzlaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1kOWEyYTEyZDNjNjBlOTVjMmE2M2RhYzM2ZWU4MDg4MjFhMThjNzE1NWU5NTQ0NTQ0Y2RkYzE5YzQwYjNmMjA3JlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZyZXNwb25zZS1jb250ZW50LXR5cGU9dmlkZW8lMkZtcDQifQ.oSauDztl7KFj7mlpUS_pzUChTRXpYWCG0S5JyGRfnRg
* **meca500_hardware smoke test** — basic robot motion through the ros2_control hardware interface
  _[video placeholder]_

## Launch & Service Commands

**Bring up MoveIt2 + RViz + the trajectory/planning-scene nodes:**
```bash
ros2 launch meca500_demo trajectory.launch.xml use_mock_hardware:=true   # sim
ros2 launch meca500_demo trajectory.launch.xml use_mock_hardware:=false  # real Meca500
```

**Set the print bed's pose** (position + orientation quaternion — use a non-identity `qx/qy/qz/qw` for the random-orientation/cube test):
```bash
ros2 service call /table_service meca500_demo/srv/Table \
  "{x: 0.25, y: 0.0, z: 0.05, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

**Send raw G-code directly** (single G1/G2/G3 moves for testing):
```bash
ros2 service call /goal_service meca500_demo/srv/Goal "{gcode: 'G1 X50 Y50 Z10 F3000'}"
ros2 service call /goal_service meca500_demo/srv/Goal "{gcode: 'G2 X50 Y0 Z10 I25 J0 F1500'}"
```

**Run the full print pipeline** (reachability sweep → parse/center/clip → print) in one shot:
```bash
ros2 launch meca500_demo print.launch.xml \
  model_file:=/path/to/model.gcode.3mf \
  out_file:=/path/to/out.txt \
  reach_csv:=/path/to/reachable_points.csv \
  layers:=21   # 0 = all layers
```

**Or run each stage of the pipeline individually:**
```bash
# 1. Sweep the reachable workspace
ros2 run meca500_demo reachability --ros-args -p out_file:=reachable_points.csv

# 2. Parse/center/clip the sliced model onto that workspace
python3 gcode/src/gcode_parser.py model.gcode.3mf out.txt \
  --reach-csv reachable_points.csv -l 21

# 3. Send the parsed file to the printer
ros2 service call /gcode_file_service meca500_demo/srv/GcodeFile "{file_path: '/path/to/out.txt'}"
```

## In Progress
* Generalizing the pipeline so any extruder can be mounted from just its URDF, with no hardcoded tool frame/offset
* Moving from mock hardware to printing on the real robot
* Reinforcement learning for adaptive tool orientation and extrusion

## Tech Stack
ROS2 | MoveIt2 (Pilz Industrial Motion Planner) | C++ | Python | Meca500 API

## Author
Rishika Bera — MS Robotics, Northwestern University
