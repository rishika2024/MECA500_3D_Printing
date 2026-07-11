# MECA500 3D Printing
A ROS2 package for controlling a Meca500 6DOF robot arm (5 μm resolution) to perform robotic 3D printing via MoveIt2.

## Overview
This project bridges the Meca500 proprietary API to MoveIt2 through a custom ROS2 hardware interface, enabling real trajectory planning and execution on physical hardware. A print pipeline sweeps the robot's reachable workspace, centers and clips sliced G-code onto the densest reachable region, and executes it move-by-move through MoveIt2's Pilz Industrial Motion Planner (LIN for straight/extruding moves, CIRC for arcs).

## Packages

* meca500_hardware — ROS2 hardware interface bridging the Meca500 API to MoveIt2
* meca500_moveit — MoveIt2 configuration and launch files
* meca500_robot — Robot description (URDF/Xacro), including the mounted extruder end-effector and `nozzle` tool frame
* meca500_demo — Reachability sweeping, planning scene setup, and the main print-execution node: parses G-code moves, plans them through Pilz LIN/CIRC, and recovers from occasional IK failures (Z-hop for travels, midpoint bisection for extruding moves) without skipping a commanded point
* gcode — G-code handling: a Python preprocessing tool (centers a sliced print on the densest reachable region, drops moves outside the workspace as gaps, validates and repairs arc geometry) plus a C++ parser library used by the trajectory node at execution time. The number of layers to print is configurable (`-l`/`layers`, see below) — the default is all layers, or pass a smaller count to print an evenly-spaced subset for a quicker test

## External Dependency: Patched Pilz Industrial Motion Planner

This project builds `pilz_industrial_motion_planner` from source (from [moveit/moveit2](https://github.com/moveit/moveit2)) instead of using the stock `apt` package, with one constant changed so its CIRC arc-fitting gate matches the Meca500's 5 μm resolution instead of the stock library's much coarser industrial-scale tolerance:

* `MAX_COLINEAR_NORM` (the near-degenerate-triangle rejection in `circleFromInterim`, `path_circle_generator.hpp`) lowered from the stock `1e-5` to `2.5e-11` (5 μm × 5 μm), so genuinely tiny print-scale arcs stop getting rejected as "no plane" errors
* the trajectory node's own flatness check (`get_arc_center`/CIRC path in `meca500_demo`) mirrors that same `2.5e-11` threshold, so an arc is only demoted to a straight line when it's below what the robot can actually resolve

The change is in [`patches/pilz_industrial_motion_planner.patch`](patches/pilz_industrial_motion_planner.patch). To set it up:
```bash
git clone https://github.com/moveit/moveit2.git
cd moveit2
git apply /path/to/Final_Project/patches/pilz_industrial_motion_planner.patch
# then colcon build the moveit_planners/pilz_industrial_motion_planner package
# into the same workspace as this repo
```

## Setup
<img width="2040" alt="Full Setup" src="https://github.com/user-attachments/assets/774f2ea0-13c4-4861-8aae-b70a6dd08630" />

## Demos

In the RViz views below, the **green** line is the `ee_trace` (every sampled end-effector position, travel and print alike) and the **purple** line is the `print_trace` (only the segments where the nozzle was actually extruding).

* **With extruder, flat bed (Benchy)** — full print pipeline of Benchy Boat

  https://private-user-images.githubusercontent.com/172546714/620307317-e0da4dfe-44fa-4f29-b36d-c69978396f66.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM3MzI0NTEsIm5iZiI6MTc4MzczMjE1MSwicGF0aCI6Ii8xNzI1NDY3MTQvNjIwMzA3MzE3LWUwZGE0ZGZlLTQ0ZmEtNGYyOS1iMzZkLWM2OTk3ODM5NmY2Ni5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNzExJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDcxMVQwMTA5MTFaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1mYjdkNTI2YTc3OWU3YzgzYmY4Mjk0ZjMwNTczMWNjNGY0MTA4ZDgzYTg2YjcwZTRlYzUzNTgwYTU0OTk4YjVkJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZyZXNwb25zZS1jb250ZW50LXR5cGU9dmlkZW8lMkZtcDQifQ.MFdStA2JyPY4ZsJx2B32xM93KCcxQoHtXvPHSJ3jU_Q
* **No extruder, random-orientation bed (cube)** — table tilted to an arbitrary pose via `/table_service`

  https://private-user-images.githubusercontent.com/172546714/620307447-fd6c1934-fea4-4575-9a14-8e8271754990.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM3MzI1NDQsIm5iZiI6MTc4MzczMjI0NCwicGF0aCI6Ii8xNzI1NDY3MTQvNjIwMzA3NDQ3LWZkNmMxOTM0LWZlYTQtNDU3NS05YTE0LThlODI3MTc1NDk5MC5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNzExJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDcxMVQwMTEwNDRaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1hNzRjY2I2ZTkxMWE1NzA2OTFhYzNiZWM1MTU2ZWJhM2ZkZTU1MjNkNDdiYjg4NjhiZjQ4NzBhMjBkZTg0MDhlJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZyZXNwb25zZS1jb250ZW50LXR5cGU9dmlkZW8lMkZtcDQifQ.SXQdC380OwjDO8CIfNpCEhOS6Pk06s2BA2w9jV7B48I
 
* **G1 (straight-line) moves**
  
  https://private-user-images.githubusercontent.com/172546714/600223649-309005d1-acc4-417c-8442-79d06b82b89f.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODAwNzI4MzksIm5iZiI6MTc4MDA3MjUzOSwicGF0aCI6Ii8xNzI1NDY3MTQvNjAwMjIzNjQ5LTMwOTAwNWQxLWFjYzQtNDE3Yy04NDQyLTc5ZDA2YjgyYjg5Zi5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNTI5JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDUyOVQxNjM1MzlaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1kOWEyYTEyZDNjNjBlOTVjMmE2M2RhYzM2ZWU4MDg4MjFhMThjNzE1NWU5NTQ0NTQ0Y2RkYzE5YzQwYjNmMjA3JlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZyZXNwb25zZS1jb250ZW50LXR5cGU9dmlkZW8lMkZtcDQifQ.oSauDztl7KFj7mlpUS_pzUChTRXpYWCG0S5JyGRfnRg
* **meca500_hardware smoke test** — basic robot motion through the ros2_control hardware interface
  
  https://private-user-images.githubusercontent.com/172546714/594355787-2bea35e4-b3d4-485b-8c0f-8fb727872e21.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM3MzIyOTcsIm5iZiI6MTc4MzczMTk5NywicGF0aCI6Ii8xNzI1NDY3MTQvNTk0MzU1Nzg3LTJiZWEzNWU0LWIzZDQtNDg1Yi04YzBmLThmYjcyNzg3MmUyMS5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNzExJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDcxMVQwMTA2MzdaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT0zZjBjNTkzZGQ4NjM1ZjBjNzA5NDU4M2QxYmE3MWFkZDRjOTZhN2RhY2YwNDdkMGFkMDU5YjViNzlhNTgyYTcxJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZyZXNwb25zZS1jb250ZW50LXR5cGU9dmlkZW8lMkZtcDQifQ.EYGsT6-cyWrjUJF8ESzELfX1IaF1rovN4gFgfdeNjbc

## Launch & Service Commands

**Bring up MoveIt2 + RViz + the trajectory/planning-scene nodes:**
```bash
ros2 launch meca500_demo trajectory.launch.xml use_mock_hardware:=true   # sim
ros2 launch meca500_demo trajectory.launch.xml use_mock_hardware:=false  # real Meca500
```

**Set the print bed's pose** (position + orientation quaternion — use a non-identity `qx/qy/qz/qw` for the random-orientation/cube test):
```bash
ros2 service call /table_service meca500_demo/srv/Table \
  "{x: 0.0, y: -0.20, z: -0.15, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
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
