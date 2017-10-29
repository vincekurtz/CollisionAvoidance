# CollisionAvoidance
Multi-Robot Collision Avoidance using Reinforcement Learning

## Dependencies
- ROS Indigo

- Stage Simulator

- Python 2.7

- scikit-learn

- numpy

## How to run
Start ROS `roscore`

Start simulation `rosrun stage_ros stageros [worldfile]`

Teleoperate robot: `python src/teleop.py`

Run super-simple autonomy: `python src/naive_controller.py`

Run simple RL controller: `python src/rl_controller.py`
