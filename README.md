# blue_soylent
blue_soylent contains a repo to control a Blue robotic arm.

 without directly interfacing with ROS. It uses [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) to communicate with Blue's ROS-based control system over a network connection.

# Control examples

1. Tele-operation, 2. Inverse Kinematic

<p align="center">
 <a href="https://www.youtube.com/watch?v=d6HUE99mb6M" target="_blank"><img src="http://img.youtube.com/vi/d6HUE99mb6M/0.jpg" alt="BLUE_SOYLENT" width="240" height="180" border="10" /></a>
 <a href="https://www.youtube.com/watch?v=i4gfILPBqGU" target="_blank"><img src="http://img.youtube.com/vi/i4gfILPBqGU/0.jpg" alt="BLUE_IK" width="240" height="180" border="10" /></a>
</p>

# Installation Procedure

### 0. Clone this repo
git clone https://github.com/yusukeurakami/blue_soylent.git

### 1. Setup basic blue environment
https://github.com/berkeleyopenarms/blue_core

Guide
https://docs.google.com/document/d/1MnyViCi9Tt03B9-3wZsePXXlKf7XiXHs0o6_c8z7at4/edit?usp=sharing

### 2. Install blue_interface with pip
```
git clone https://github.com/berkeleyopenarms/blue_interface.git
cd blue_interface
pip install -e .
```
### 3. Install Leap motion
Follow these.

https://forums.leapmotion.com/t/linux-install-of-sdk-fails/5158/6

https://github.com/robotlearn/pyrobolearn/blob/master/pyrobolearn/tools/interfaces/sensors/install_leapmotion_ubuntu.txt

### 4. Boot robot
Run either of follows and boot the real robot or the one in Gazebo.

#### Real-robot
`roslaunch blue_bringup right.launch param_file:=/path/to/blue_configs/blue_right_v2.yaml`

#### Gazebo
`roslaunch blue_gazebo right.launch`

### 5. run Example scripts (`blue_soylent/examples`)
  - IK control
    `python leap_controller.py --IK`

  - Tele-operation
    `python leap_controller.py`


### API Documentation
https://blue-interface.readthedocs.io/en/latest/
