# Vacuum Robot

ROS 2 and Docker-based Raspberry Pi vacuum robot project with LiDAR mapping, Arduino motor control, and a future NUEVO web UI dashboard.

## System
- Raspberry Pi 4 running Ubuntu 24.04
- Docker for ROS 2 Jazzy environment
- ROS 2 nodes for robot software
- Arduino for motor control and encoder feedback
- LiDAR for mapping and navigation

## Planned Architecture
- LiDAR Node publishes `/scan`
- Robot Node publishes `/cmd_vel`
- Bridge Node communicates with Arduino
- NUEVO UI displays robot status in browser
