# MECA500 3D Printing
A ROS2 workspace for driving a Meca500 6-DOF robot arm (5 μm resolution) as a 3D printer, through MoveIt2.

## Overview
This project bridges the Meca500 proprietary API to MoveIt2 through a custom ROS2 hardware interface, enabling real trajectory planning and execution on physical hardware. A print pipeline locates the bed (from a nozzle touch probe), sweeps the robot's reachable workspace, centers and clips sliced G-code onto the densest reachable region, groups the moves into batches by type, and runs each batch as one blended MoveIt2 Pilz motion sequence (LIN for straight/extruding moves, CIRC for arcs) — after a pre-planning pass over the whole print. Everything hardware- or print-specific lives in three YAML files (see [Configuration](#configuration)).

## Packages

All first-party packages are prefixed `msr_` to keep them apart from vendor packages.

* **msr_meca500_hardware** — ros2_control hardware interface (`Meca500System`) bridging the Meca500 TCP API to MoveIt2
* **msr_meca500_robot** — robot description (URDF/Xacro): the arm, the mounted Ender3 extruder with its `nozzle` tool frame, and the Ender3 chassis/bed as environment collision geometry
* **msr_meca500_moveit** — MoveIt2 configuration and launch files
* **msr_gcode** — G-code handling: a Python preprocessing tool (centers a sliced print on the densest reachable region, drops moves outside the workspace as gaps, validates and repairs arc geometry) plus a C++ parser library used at execution time. Layer count is configurable (`-l`/`layers`) — default all layers, or a smaller count for an evenly-spaced subset
* **msr_meca500_print_pipeline** — the print application. Nodes:
  * `gcode_print_executor` — the executor: groups G-code into batches by move type, plans and runs each batch as one blended Pilz LIN/CIRC sequence, drives the Ender3 over serial (temps, extrusion, bed re-home), and recovers from IK failures (Z-hop for travels, midpoint bisection for extruding moves) without skipping a commanded point
  * `reachability` — sweeps an N×N grid over the bed, writes the reachable points to CSV
  * `planningscene` — hosts `/table_service`, publishes the bed pose as `/table_marker`
  * `bed_from_touches` — fits the bed plane from nozzle touch-probe joint poses (or applies a flat default) and pushes it to `/table_service`
* **msr_meca500_rl** — experimental: RL for adaptive tool orientation / extrusion

## External Dependency: Patched Pilz Industrial Motion Planner

This project builds `pilz_industrial_motion_planner` from source (from [moveit/moveit2](https://github.com/moveit/moveit2)) instead of using the stock `apt` package, with one constant changed so its CIRC arc-fitting gate matches the Meca500's 5 μm resolution instead of the stock library's much coarser industrial-scale tolerance:

* `MAX_COLINEAR_NORM` (the near-degenerate-triangle rejection in `circleFromInterim`, `path_circle_generator.hpp`) lowered from the stock `1e-5` to `2.5e-11` (5 μm × 5 μm), so genuinely tiny print-scale arcs stop getting rejected as "no plane" errors
* `gcode_print_executor`'s own flatness check (`get_arc_center`/CIRC path) mirrors that same `2.5e-11` threshold, so an arc is only demoted to a straight line when it's below what the robot can actually resolve

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

### Build

Needs ROS 2 Kilted with MoveIt 2, plus the patched Pilz planner (above) built into the same workspace.

```bash
# from your workspace's src/
git clone <this repo> Final_Project
rosdep install --from-paths Final_Project --ignore-src -r -y   # rclcpp, moveit, python3-serial, python3-yaml, ...
cd ..
colcon build --symlink-install
source install/setup.bash
```

Build order (`msr_gcode` + `msr_meca500_robot` → `msr_meca500_hardware` → `msr_meca500_moveit` → `msr_meca500_print_pipeline`) is resolved by colcon.

### First-time setup

1. Fill in `msr_meca500_print_pipeline/config/machine_settings.yaml` for your printer — serial port, `M503` E-steps, `M114` home position, hotend/bed temps, the nozzle tip offset.
2. Locate the bed (see [Configuration](#configuration)) and write the touch poses into `bed_settings.yaml`, or leave `default_bed: true` for a flat bed at a known spot.

## Configuration

`msr_meca500_print_pipeline/config/` holds three ROS 2 params files, loaded by the
launch files (`<param from>`) and read directly by the Ender3 sequence scripts.
Split by *who sets each value and when*:

| file | contents | you touch it... |
|---|---|---|
| `machine_settings.yaml` | serial port, baud, E-steps/mm, home XY + feedrate, temps, extruder link names + tip offsets | once, at bring-up |
| `bed_settings.yaml` | `default_bed`, `default_bed_pose`, and the nozzle **touch poses** the plane is fit from | whenever the bed moves — output of the touch procedure below |
| `print_tuning.yaml` | run-mode defaults, reachability grid, extrusion floor + feed rates, re-home constants | rarely; only if a comment tells you to |

**Locating the bed** (`default_bed:=false`): jog the nozzle to touch the bed at
≥3 spots, and once at the centre. For each, record the `position` list from
`ros2 topic echo /joint_states` into `bed_settings.yaml` (`bed_touch_poses`
flattened 6-at-a-time, `bed_center_pose` for the centre). `bed_from_touches`
runs FK on each to get the nozzle tip, fits the plane by SVD, and sets it on
`/table_service`. With `default_bed:=true` it just uses `default_bed_pose`.

## Demos

In the RViz views below, the **green** line is the `ee_trace` (every sampled end-effector position) and the **purple** line is the `print_trace` (only the segments where the nozzle was actually extruding).

* **With extruder, flat bed (Benchy)** — full print pipeline of Benchy Boat, no. of layers printed = 21

  https://private-user-images.githubusercontent.com/172546714/620307317-e0da4dfe-44fa-4f29-b36d-c69978396f66.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM3MzI0NTEsIm5iZiI6MTc4MzczMjE1MSwicGF0aCI6Ii8xNzI1NDY3MTQvNjIwMzA3MzE3LWUwZGE0ZGZlLTQ0ZmEtNGYyOS1iMzZkLWM2OTk3ODM5NmY2Ni5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNzExJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDcxMVQwMTA5MTFaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1mYjdkNTI2YTc3OWU3YzgzYmY4Mjk0ZjMwNTczMWNjNGY0MTA4ZDgzYTg2YjcwZTRlYzUzNTgwYTU0OTk4YjVkJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZyZXNwb25zZS1jb250ZW50LXR5cGU9dmlkZW8lMkZtcDQifQ.MFdStA2JyPY4ZsJx2B32xM93KCcxQoHtXvPHSJ3jU_Q
* **No extruder, random-orientation bed (cube)** — table tilted to an arbitrary pose via `/table_service`, no. of layers printed = 7

  https://private-user-images.githubusercontent.com/172546714/620307447-fd6c1934-fea4-4575-9a14-8e8271754990.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM3MzI1NDQsIm5iZiI6MTc4MzczMjI0NCwicGF0aCI6Ii8xNzI1NDY3MTQvNjIwMzA3NDQ3LWZkNmMxOTM0LWZlYTQtNDU3NS05YTE0LThlODI3MTc1NDk5MC5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNzExJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDcxMVQwMTEwNDRaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1hNzRjY2I2ZTkxMWE1NzA2OTFhYzNiZWM1MTU2ZWJhM2ZkZTU1MjNkNDdiYjg4NjhiZjQ4NzBhMjBkZTg0MDhlJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZyZXNwb25zZS1jb250ZW50LXR5cGU9dmlkZW8lMkZtcDQifQ.SXQdC380OwjDO8CIfNpCEhOS6Pk06s2BA2w9jV7B48I
 
* **G1 (straight-line) moves**
  
  https://private-user-images.githubusercontent.com/172546714/600223649-309005d1-acc4-417c-8442-79d06b82b89f.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODAwNzI4MzksIm5iZiI6MTc4MDA3MjUzOSwicGF0aCI6Ii8xNzI1NDY3MTQvNjAwMjIzNjQ5LTMwOTAwNWQxLWFjYzQtNDE3Yy04NDQyLTc5ZDA2YjgyYjg5Zi5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNTI5JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDUyOVQxNjM1MzlaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1kOWEyYTEyZDNjNjBlOTVjMmE2M2RhYzM2ZWU4MDg4MjFhMThjNzE1NWU5NTQ0NTQ0Y2RkYzE5YzQwYjNmMjA3JlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZyZXNwb25zZS1jb250ZW50LXR5cGU9dmlkZW8lMkZtcDQifQ.oSauDztl7KFj7mlpUS_pzUChTRXpYWCG0S5JyGRfnRg
* **msr_meca500_hardware smoke test** — basic robot motion through the ros2_control hardware interface
  
  https://private-user-images.githubusercontent.com/172546714/594355787-2bea35e4-b3d4-485b-8c0f-8fb727872e21.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM3MzIyOTcsIm5iZiI6MTc4MzczMTk5NywicGF0aCI6Ii8xNzI1NDY3MTQvNTk0MzU1Nzg3LTJiZWEzNWU0LWIzZDQtNDg1Yi04YzBmLThmYjcyNzg3MmUyMS5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNzExJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDcxMVQwMTA2MzdaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT0zZjBjNTkzZGQ4NjM1ZjBjNzA5NDU4M2QxYmE3MWFkZDRjOTZhN2RhY2YwNDdkMGFkMDU5YjViNzlhNTgyYTcxJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZyZXNwb25zZS1jb250ZW50LXR5cGU9dmlkZW8lMkZtcDQifQ.EYGsT6-cyWrjUJF8ESzELfX1IaF1rovN4gFgfdeNjbc

## Launch & Service Commands

**System bring-up** — MoveIt2 + ros2_control + RViz + `planningscene` + `bed_from_touches` + `gcode_print_executor`. Run once, leave up:
```bash
ros2 launch msr_meca500_print_pipeline main.launch.xml use_mock_hardware:=true   # sim
ros2 launch msr_meca500_print_pipeline main.launch.xml use_mock_hardware:=false  # real Meca500
# default_bed:=false to fit the bed from bed_settings.yaml instead of the flat default
```

**Run a print** — set bed → reachability sweep → parse/center/clip → execute. Needs `main.launch.xml` already running (it hosts the services):
```bash
ros2 launch msr_meca500_print_pipeline print.launch.xml \
  model_file:=/path/to/model.gcode.3mf \
  out_file:=/path/to/out.txt \
  layers:=21          # 0 = all layers
  # default_bed:=false to re-fit the bed for this print
```

**Set the bed pose manually** (e.g. a deliberately tilted bed for the cube demo):
```bash
ros2 service call /table_service msr_meca500_print_pipeline/srv/Table \
  "{x: 0.0, y: -0.20, z: -0.15, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

**Send raw G-code directly** (single G1/G2/G3 moves for testing):
```bash
ros2 service call /goal_service msr_meca500_print_pipeline/srv/Goal "{gcode: 'G1 X50 Y50 Z10 F3000'}"
ros2 service call /goal_service msr_meca500_print_pipeline/srv/Goal "{gcode: 'G2 X50 Y0 Z10 I25 J0 F1500'}"
```

**Run each stage individually** (`<config>` = `$(ros2 pkg prefix msr_meca500_print_pipeline)/share/msr_meca500_print_pipeline/config`):
```bash
# 1. Set the bed pose
ros2 run msr_meca500_print_pipeline bed_from_touches --ros-args \
  --params-file <config>/machine_settings.yaml --params-file <config>/bed_settings.yaml

# 2. Sweep the reachable workspace
ros2 run msr_meca500_print_pipeline reachability --ros-args \
  --params-file <config>/machine_settings.yaml --params-file <config>/print_tuning.yaml \
  -p out_file:=reachable_points.csv

# 3. Parse/center/clip the sliced model onto that workspace
python3 msr_gcode/src/gcode_parser.py model.gcode.3mf out.txt --reach-csv reachable_points.csv -l 21

# 4. Send the parsed file to the executor
ros2 service call /gcode_file_service msr_meca500_print_pipeline/srv/GcodeFile "{file_path: '/path/to/out.txt'}"
```

## Tech Stack
ROS2 | MoveIt2 | Pilz Industrial Motion Planner | C++ | Python | Meca500 API

## Author
Rishika Bera — MS Robotics, Northwestern University
