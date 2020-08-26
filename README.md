# Overview
This package enables the creation of ROS robot environments in OpenAI Gym. 

This is a fork of the original openai_ros package from The Construct, found here:
https://bitbucket.org/theconstructcore/openai_ros.git

# Installation
Installation is similar to other ROS packages. Simply clone it into a catkin workspace and build.

Execute the following commands:<br>
`cd ~/ros_ws/src`<br>
`git clone https://github.com/roboav8r/openai_ros`<br>
`cd ~/ros_ws`<br>
`catkin_make`<br>
`source devel/setup.bash`<br>
`rosdep install openai_ros`<br>

# New environments
The utexas fork is meant to represent UT-specific robots and environments for training of ROS robots in Gazebo simulation. As of now, there are two new robot environments and three new task environments, described below.

## Robot environments
- walrus: A robot environment for the Walrus robot. It launches a typical Walrus robot with two LIDAR scanning rangefinders, an IMU, and odometry. 
The file is stored at robot_envs/walrus_env.py

- walrus_upright: A robot environment for the Walrus robot, which spawns the robot in an upright position. This is used for the self-balancing task described below. The robot has the same sensor suite as in walrus_env, and the only difference is the spawn orientation as specified by the launch file.
The file is stored at robot_envs/walrus_upright_env.py

Both Walrus environments depend on the walrus_description package, openai branch here:
https://github.com/UTNuclearRobotics/walrus_description/tree/openai

## Task Environments
- WalrusBalance-v0 - An inverted pendulum/self-balancing robot task.
  - Defined in task_envs/walrus/walrus_balance.py
  - Parameters in task_envs/walrus/config/walrus_balance.yaml
  - Observations
    - 16 Scans: 8 each from the LIDAR scan messages on /scan and /scan_1 topics.
    - 2 IMU measurements. Pitch attitude (imu/data/orientation/y) and pitch rate (imu/data/angular_velocity/y)
    - 1 Odometry measurement: Horizontal position (/odom/pose/pose/position/x) 
  - Actions
    - Commands /cmd_vel/linear/x velocity, with speed range defined by [linear_speed_(min/max)] values in the .yaml file 
  - Rewards
    - [stay_up_reward] value is awarded each timestep.
    - [position_penalty] is subtracted for every meter of nonzero position in x. For example, a penalty of 10 results in -10 reward if the x-position is 1m.
    - [ang_velocity_reward] is designed to keep the rotation slow, and avoid jerky or sudden movements. It is awarded when the pitch velocity is less than [ang_velocity_threshold]. 
  - Completion conditions
    - "Crash" when robot acceleration exceeds [max_linear_acceleration] parameter.
    - "Rollover" when pitch attitude (imu/data/orientation/y) is out of bounds of [min_pitch_orient, max_pitch_orient] OR
    - Pitch rate (imu/data/angular_velocity/y) is out of bounds of [min_pitch_rate, max_pitch_rate]

- WalrusStairs-v0 - A task environment designed to teach the robot to climb and descend stairs without rolling over.

- WalrusNav-v0 - a simple 2D nav task. 
  - NOTE: sometimes the barriers in the clearpath_playpen environment spawn in an incorrect orientation. Needs to be fixed.
  - Defined in task_envs/walrus/walrus.nav.py
  - Parameters in task_envs/walrus/config/walrus_nav.yaml
  - Observations
    - 16 Scans: 8 each from the LIDAR scan messages on /scan and /scan_1 topics.
    - 1 yaw orientation measurement from imu/data/orientation/z
    - 2 Odometry measurements to describe the 2D position: /odom/pose/pose/position/x and /odom/.../position/y
  - Actions
    - Commands /cmd_vel/linear/x and /cmd_vel/angular/y velocities, with speed range defined by linear_speed and angular_speed values in the .yaml file 
  - Rewards
    - [stay_alive_reward] value is awarded each timestep.
    - [forward_velocity_reward] is given as a multiple of linear speed. If this reward value is positive, forward motion gives a reward, and rear motion gives a penalty.
    - [position_reward]/(distance to goal in m) is awarded at each timestep. For example, a value of 10 gives a reward of 1 if the robot is 10m from the goal, and a reward of 10 if the robot is 1m away from the goal.
    - [goal_reached_reward] is given if the robot position is within [success_radius] meters of [x_goal, y_goal].
  - Completion conditions
    - "Crash" when robot acceleration exceeds [max_linear_acceleration] parameter.
    - "Out of bounds" when robot exceeds [(min/max)_(x/y)_disp] parameters.
    - Robot position is within [success_radius] meters of [x_goal, y_goal].
