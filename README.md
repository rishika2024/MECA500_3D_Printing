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

## Demo: Robot Moving Based On G1 Inputs
https://private-user-images.githubusercontent.com/172546714/600223649-309005d1-acc4-417c-8442-79d06b82b89f.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODAwNzI4MzksIm5iZiI6MTc4MDA3MjUzOSwicGF0aCI6Ii8xNzI1NDY3MTQvNjAwMjIzNjQ5LTMwOTAwNWQxLWFjYzQtNDE3Yy04NDQyLTc5ZDA2YjgyYjg5Zi5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNTI5JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDUyOVQxNjM1MzlaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1kOWEyYTEyZDNjNjBlOTVjMmE2M2RhYzM2ZWU4MDg4MjFhMThjNzE1NWU5NTQ0NTQ0Y2RkYzE5YzQwYjNmMjA3JlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZyZXNwb25zZS1jb250ZW50LXR5cGU9dmlkZW8lMkZtcDQifQ.oSauDztl7KFj7mlpUS_pzUChTRXpYWCG0S5JyGRfnRg

## In Progress
* Generalizing the pipeline so any extruder can be mounted from just its URDF, with no hardcoded tool frame/offset
* Moving from mock hardware to printing on the real robot
* Reinforcement learning for adaptive tool orientation and extrusion

## Tech Stack
ROS2 | MoveIt2 (Pilz Industrial Motion Planner) | C++ | Python | Meca500 API

## Author
Rishika Bera — MS Robotics, Northwestern University
