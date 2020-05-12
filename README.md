# blue_soylent
blue_soylent contains a repo to control a Blue robotic arm.

 without directly interfacing with ROS. It uses [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) to communicate with Blue's ROS-based control system over a network connection.

### Why use blue_interface?
- No dependency on ROS
- Easy to connect to multiple robots
- Works with both Python 2 and 3
- Works with Mac, Windows, and Linux
- Works in Jupyter Notebooks

### 1. Setup basic blue environment

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


### Examples (`blue_interface/examples`)
  - `gripper_controller.py` - An example of opening and closing Blue's gripper.
  - `inverse_kinematics.py` - An example of sending Blue an end effector pose command.
  - `print_status.py` - An example of reading state and printing values from Blue.
  - `zero_position.py` - An example of sending Blue a command in joint space.

### API Documentation
https://blue-interface.readthedocs.io/en/latest/
