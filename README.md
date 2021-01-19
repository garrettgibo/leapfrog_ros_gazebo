# Leapfrog ROS Gazebo

Leapfrog ROS Gazebo is a ROS base implementation of the LEAPFROG vehicle that 
runs in Gazebo.

## Installation

1. Install and initialize ROS Noetic and catkin-tools

```sh
#### ROS Noetic
# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Installation
sudo apt update
sudo apt install ros-noetic-desktop-full

# Environment Setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

#### catkin_tools
sudo apt install python-catkin-tools
```

2. Create ROS workspace
```sh
# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# Configure workspace to extend ROS noetic
catkin config --extend /opt/ros/noetic

git clone https://github.com/garrettgibo/leapfrog_ros_gazebo src/leapfrog_ros_gazebo
```

3. Build Workspace
```sh
cd ~/catkin_ws
catkin build
```

4. Update Packages and Paths
```sh
source devel/setup.bash
```

## Usage

1. Launch simulator with LEAPFROG lander and basic gazebo interface plugin setup

```sh
roslaunch leapfrog_ros_gazebo leapfrog_world.launch
```

2. Controlling Vehicle is done by sending a message to these topics:

- `/lander/engine`
  - Type: std_msgs/Float32
- `/lander/thrusters`
  - Type: std_msgs/Float32MultiArray
- `/lander/actuators`
  - Type: std_msgs/Float32MultiArray
  
Verify functionality of simulation and gazebo interface
```sh
rostopic pub /lander/thrusters std_msgs/Float32MultiArray \
"layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [10000, 10000, 10000, 10000]"
```
