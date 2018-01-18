# CollisionAvoidance
Multi-Robot Collision Avoidance using Reinforcement Learning

## Dependencies
- ROS Kinetic

- Stage-ros 
    - with pose subscriber (for teleportation) (https://github.com/ros-simulation/stage_ros/tree/add_pose_sub)
    - with modification to publish collision info (https://answers.ros.org/question/65686/detect-collision-stage/)

- Python 2.7

- tensorflow

- numpy

## How to run
Start ROS `roscore`

Run simple RL controller (will start simulation too): `rosrun collision_avoidance rl_controller.py`

Test a trained model `rosrun collision_avoidance test_trained_model.py`

Start simulation alone: `rosrun stage_ros stageros [worldfile]`

Teleoperate robot: `rosrun collision_avoidance teleop.py`

Run super-simple autonomy: `python collision_avoidance naive_controller.py`
