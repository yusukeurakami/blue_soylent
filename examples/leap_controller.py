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

listener = SampleListener()
controller = Leap.Controller()

blue.calibrate_gripper()

goal_hist = np.array(7)
fig = plt.figure()
axes = fig.add_subplot(111)
i = 0
_rate = 0.01

while True:
    hands_data = listener.get_hand(controller)
    # listener.on_frame(controller)
    # listener.on_rate = 0.01_data.keys())
    # print(if hands_data["Right hand"])
    # print("Right hand" in hands_data.keys())
    # frame = controller.frame()
    # print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
    #         frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

    # IK = False
    ## IK approach
    if args.IK:
        if  "Right hand" in hands_data.keys():
            hand_data = hands_data["Right hand"]
            pos = hand_data['palm_position']
            ori = [hand_data['palm_pitch'], hand_data['palm_roll'], hand_data['palm_yaw']]
            grab_strength = hand_data['grab_strength']
            # print()
            # print(">>> ori before: ", [x/3.14*180 for x in ori])
            target_position = [x/1000 for x in pos] # x, y, z
            pos[0], pos[1], pos[2] = pos[2], pos[0], pos[1] # z x y to x y z
            ori[0], ori[1], ori[2] = ori[2], -ori[0]+3.14, ori[1] # z y x to x y z
            # print(ori)
            # ori = [ori[0],3.14,0]
            # print(">>> ori after", [x/3.14*180 for x in ori])

            target_position[0] -= 0.4
            target_position[2] += 0.3
            target_orientation = list(euler2quat(ori)) # w, x, y, z
            # target_orientation = to[1:] + to[:1]
            # Compute IK solution
            goal_curr = blue.inverse_kinematics(target_position, target_orientation)
            # Send command to robot
            if goal_curr != []:
                # alpha = 0.5
                # goal = goal_hist*alpha + goal_curr*(1-alpha)
                # goal_hist = goal
                # plt.scatter(i, goal[0], c='r')
                # plt.scatter(i, goal[1], c='g')
                # plt.scatter(i, goal[2], c='b')
                # plt.scatter(i, goal[3], c='r')
                # plt.scatter(i, goal[4], c='r')
                # plt.scatter(i, goal[5], c='g')
                # plt.scatter(i, goal[6], c='b')
                # plt.pause(0.01)
                goal = goal_curr

                print("goal: ", goal)
                blue.set_joint_positions(goal, duration=_rate, soft_position_control=False)
                blue.command_gripper(grab_strength, 10.0, wait=False)
    else:
        if  "Right hand" in hands_data.keys():
            hand_data = hands_data["Right hand"]
            pos = hand_data['palm_position']
            ori = [hand_data['palm_pitch'], hand_data['palm_roll'], hand_data['palm_yaw']]
            grab_strength = hand_data['grab_strength']
            # Send command to robot
            goal = np.array([0.0, -0.85, 1.571, 0, -1.571, -0.2, 0.0])
            # goal = np.zeros(7)
            # alpha = 0.5
            # goal = goal_hist*alpha + goal_curr*(1-alpha)
            # goal_hist = goal

            pos[0], pos[1], pos[2] = pos[2], pos[0], pos[1] # z x y to x y z
            ori[0], ori[1], ori[2] = ori[2], ori[0], ori[1] # z y x to x y z
            target_position = [x/1000 for x in pos] # x, y, z
            target_position[0] += 0.05
            target_position[2] -= 0.2

            # orientation
            goal[0] += (ori[0]*1 + target_position[1]*1.5) # shoulder dir
            goal[4] += ori[2]       # arm twist
            goal[5] += ori[1]*2     # wrist up down
            goal[6] += ori[2]       # wrist twist

            # height
            goal[1] += target_position[2]*5
            goal[3] -= target_position[2]*5

            # stretch
            goal[3] -= target_position[0]*10

            print("goal: ", goal)
            # blue.set_joint_positions(goal, duration=0.005, soft_position_control=True)
            blue.set_joint_positions(goal, duration=0.0025, soft_position_control=False)

        if  "Left hand" in hands_data.keys():
            hand_data = hands_data["Left hand"]
            pos = hand_data['palm_position']
            ori = [hand_data['palm_pitch'], hand_data['palm_roll'], hand_data['palm_yaw']]
            grab_strength = hand_data['grab_strength']
            blue.command_gripper(ori[1], 20.0, wait=False)

    # Wait for system to settle
    i+=1
    # time.sleep(0.05)
    time.sleep(0.025)


plt.show()

