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
### walrus: A robot environment for the Walrus robot. It launches a typical Walrus robot with two LIDAR scanning rangefinders, an IMU, and odometry. 
The file is stored at robot_envs/walrus_env.py

### walrus_upright: A robot environment for the Walrus robot, which spawns the robot in an upright position. This is used for the self-balancing task described below. The robot has the same sensor suite as in walrus_env, and the only difference is the spawn orientation as specified by the launch file.
The file is stored at robot_envs/walrus_upright_env.py

Both Walrus environments depend on the walrus_description package, openai branch here:
https://github.com/UTNuclearRobotics/walrus_description/tree/openai

## Task Environments
### WalrusBalance
