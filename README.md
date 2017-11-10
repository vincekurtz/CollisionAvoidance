# CollisionAvoidance
Multi-Robot Collision Avoidance using Reinforcement Learning

## Dependencies
- ROS Indigo

- Stage-ros with pose subscriber (for teleportation) (https://github.com/ros-simulation/stage\_ros/tree/add\_pose\_sub)

- Python 2.7

- scikit-learn

- numpy

## How to run
Start ROS `roscore`

Run simple RL controller (will start simulation too): `rosrun collision_avoidance /rl_controller.py`

Start simulation alone: `rosrun stage_ros stageros [worldfile]`

Teleoperate robot: `rosrun collision_avoidance teleop.py`

Run super-simple autonomy: `python collision_avoidance naive_controller.py`
