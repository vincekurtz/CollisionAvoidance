# CollisionAvoidance
Multi-Robot Collision Avoidance using Reinforcement Learning

## Dependencies
ROS Indigo
Stage Simulator

## How to run
Start ROS `roscore`
Start simulation `rosrun stage_ros stageros [worldfile]`
Teleoperate robot\_0 `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=robot_0/cmd_vel`
