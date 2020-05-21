#!/usr/bin/env python2

# A basic example of sending Blue a command in cartesian space.
from blue_interface import BlueInterface
import numpy as np
import time
import sys
import argparse

import Leap
from utils.rotations import quat2euler, euler2quat, mat2euler
from utils.leap_listener import SampleListener

import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='switch the control mode')
parser.add_argument('--IK', default=False, action='store_true',
                    help='switch to IK-control')
args = parser.parse_args()

side = "right"
ip = "127.0.0.1"
blue = BlueInterface(side, ip)
# Initialize the blue gripper
blue.calibrate_gripper()

# Leap Motion
listener = SampleListener()
controller = Leap.Controller()


target_angles_init = np.array([0.0, -0.85, 1.571, 0, -1.571, -0.2, 0.0])
target_angles_hist = target_angles_init.copy()

i = 0
while True:
    hands_data = listener.get_hand(controller)

    ## IK approach
    if args.IK:
        if  "Right hand" in hands_data.keys():
            hand_data = hands_data["Right hand"]
            pos = hand_data['palm_position']
            ori = [hand_data['palm_pitch'], hand_data['palm_roll'], hand_data['palm_yaw']]
            grab_strength = hand_data['grab_strength']
            target_position = [x/1000 for x in pos] # x, y, z
            pos[0], pos[1], pos[2] = pos[2], pos[0], pos[1] # z x y to x y z
            ori[0], ori[1], ori[2] = ori[2], -ori[0]+3.14, ori[1] # z y x to x y z

            # Adjust the offset
            target_position[0] -= 0.4
            target_position[2] += 0.3
            target_orientation = list(euler2quat(ori)) # w, x, y, z
            # target_orientation = target_orientation[1:] + target_orientation[:1]

            # Compute IK solution
            goal_curr = blue.inverse_kinematics(target_position, target_orientation)
            # Send command to robot
            if goal_curr != []:
                goal = goal_curr
                print("goal: ", goal)
                blue.set_joint_positions(goal, duration=3, soft_position_control=False)
                blue.command_gripper(grab_strength, 10.0, wait=False)

        # Wait for system to settle
        i+=1
        time.sleep(3)

    # Direct motor angle mapping approach
    else:
        if  "Right hand" in hands_data.keys():
            hand_data = hands_data["Right hand"]
            pos = hand_data['palm_position']
            ori = [hand_data['palm_pitch'], hand_data['palm_roll'], hand_data['palm_yaw']]
            grab_strength = hand_data['grab_strength']

            pos[0], pos[1], pos[2] = pos[2], pos[0], pos[1] # z x y to x y z
            ori[0], ori[1], ori[2] = ori[2], ori[0], ori[1] # z y x to x y z
            target_position = [x/1000 for x in pos] # x, y, z
            target_position[0] += 0.05
            target_position[2] -= 0.2

            # Pre-defined Initial position of the robot
            target_angles = target_angles_init.copy()

            # orientation
            target_angles[0] += (ori[0]*1 + target_position[1]*1.5) # shoulder dir
            target_angles[4] += ori[2]       # arm twist
            target_angles[5] += ori[1]*2     # wrist up down
            target_angles[6] += ori[2]       # wrist twist

            # height
            target_angles[1] += target_position[2]*5
            target_angles[3] -= target_position[2]*5

            # depth direction stretch
            target_angles[3] -= target_position[0]*10

            smoothening = True
            if smoothening:
                alpha = 0.9
                target_angles = target_angles*(1-alpha) + target_angles_hist*alpha
                target_angles_hist = target_angles

            # Send command to robot
            print("target_angles: ", target_angles)
            blue.set_joint_positions(target_angles, duration=0.0025, soft_position_control=False)

        if  "Left hand" in hands_data.keys():
            hand_data = hands_data["Left hand"]
            pos = hand_data['palm_position']
            ori = [hand_data['palm_pitch'], hand_data['palm_roll'], hand_data['palm_yaw']]
            grab_strength = hand_data['grab_strength']
            blue.command_gripper(ori[1], 20.0, wait=False)

        # Wait for system to settle
        i+=1
        time.sleep(0.025)

