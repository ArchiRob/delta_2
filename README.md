# Stewart platform inverse kinematics

This repo contains the code needed to run and simulate a 6DoF parallel manipulator based on a Stewart platform with rotary linkages. The layout of the linkages, in parallel pairs is designed to mimic the previous 3DoF delta manipulator design for easy integration with existing systems.

The implementation here is based on the work in https://github.com/daniel-s-ingram/stewart which demonstrates the use of .sdf files with ROS.

A .sdf file is used to describe the robot due to the ability to handle closed kinematic chains (and is easier to integrate with UAV models from the PX4-Autopilot stack). 

The inverse kinematics equations are taken from https://www.cambridge.org/core/journals/robotica/article/kinematic-and-dynamic-analysis-of-stewart-platformbased-machine-tool-structures/44227E02990F830098CB3897F3AED707 and adapted to rotary joints.

Gazebo models are available for a 6DOF stewart platform based manipulator and a regular 3DOF delta robot. Beware sending non-unit orientation quaternions to the delta robot will not show errors but will muck up the inverse kinematics.

## Setup instructions
Clone this repo in your workspace and build:
```
cd ~/catkin_ws/src
git clone https://github.com/lachie-aerialrobotics/delta_2.git
catkin build
```
To test in gazebo, first update any changes to the .sdf:
```
cd gazebo/models/stewart_platform
erb model.sdf.erb > model.sdf
```
Then build the motor plugin:
```
cd gazebo/plugin
mkdir build
cd build
cmake ..
make
```

Then run the simulation:
```
roslaunch delta_2 manipulator_sim.launch type:="stewart"
```
or
```
roslaunch delta_2 manipulator_sim.launch type:="delta"
```
Use dynamic reconfigure to send setpoints to the manipulator

