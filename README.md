# MECA500 3D Printing
A ROS2 package for controlling a Meca500 6DOF robot arm (5 μm resolution) to perform robotic 3D printing via MoveIt2.

## Overview
This project bridges the Meca500 proprietary API to MoveIt2 through a custom ROS2 hardware interface, enabling real trajectory planning and execution on physical hardware. A Python pipeline parses G Code into end effector waypoints for direct MoveIt2 execution.

## Packages

* meca500_hardware — ROS2 hardware interface bridging the Meca500 API to MoveIt2
* meca500_moveit — MoveIt2 configuration and launch files
* meca500_robot — Robot description (URDF/Xacro)
* meca500_demos — Demo launch files and example scripts

## Demo

https://private-user-images.githubusercontent.com/172546714/594355787-2bea35e4-b3d4-485b-8c0f-8fb727872e21.mp4?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NzkxNDI0NzksIm5iZiI6MTc3OTE0MjE3OSwicGF0aCI6Ii8xNzI1NDY3MTQvNTk0MzU1Nzg3LTJiZWEzNWU0LWIzZDQtNDg1Yi04YzBmLThmYjcyNzg3MmUyMS5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjYwNTE4JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI2MDUxOFQyMjA5MzlaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1iNTk4YTZiM2Y3OGVhYzIwNmNjYmQ2YmU5M2IzYjdkODM4M2Q5YThmOTRlZjhjMjdjOTlmYjc2NmJlZTA5OWQzJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZyZXNwb25zZS1jb250ZW50LXR5cGU9dmlkZW8lMkZtcDQifQ.RNDSiFRcbi8psIV65q9iDcPFrKnLXqOtMR177ydP8xs

## In Progress
* Visualization of 3D print path in MoveIt2
* Benchy boat print test (target: 2 to 3 weeks)
* Reinforcement learning for adaptive tool orientation and extrusion

## Tech Stack
ROS2 | MoveIt2 | C++ | Python | Meca500 API

## Author
Rishika Bera — MS Robotics, Northwestern University
