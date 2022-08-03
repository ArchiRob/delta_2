# Stewart platform kinematics

This repo contains the code needed to run and simulate a 6DoF parallel manipulator based on a Stewart platform with rotary linkages. The layout of the linkages, in parallel pairs is designed to mimic the previous 3DoF delta manipulator design for easy integration with existing systems.

## Setup instructions
Clone this repo in `~/catkin_ws/src`:
```
cd ~/catkin_ws/src
git clone https://github.com/lachie-aerialrobotics/delta_2.git
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

Gazebo 