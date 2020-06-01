#!/usr/bin/env python3

# A basic example of using BlueInterface for joint positions control.

from blue_interface import BlueInterface
import numpy as np
import cv2
from skimage.transform import resize
from threading import Thread, Lock
from gym import core, spaces

CAMERA_ID = 0

class BlueRobotEnv(object):
    def __init__(self, from_pixels=True, height=100, width=100, camera_id=0, control="torque"):
        side = "right"
        ip = "127.0.0.1"
        self.blue = BlueInterface(side, ip)
        self.blue.calibrate_gripper()
        self._from_pixels = from_pixels
        self.control = control
        self._frame_skip = frame_skip
        self.channels_first=True
        
        self.camera = StreamVideo(height=height, width=width, camera_id=camera_id)
        self.camera.start()

        # true and normalized action spaces
        self._true_action_space = spaces.Box(
            low=[-3.3999, -2.2944, -2.6761, -2.2944, -2.6761, -2.2944, -2.6761, 0.0],
            high=[2.3412, 0, 2.6761, 0, 2.6761, 0, 2.676, 0.0],
            shape=np.zeros(8),
            dtype=np.float32
        )
        self._norm_action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=self._true_action_space.shape,
            dtype=np.float32
        )

        # create observation space
        if from_pixels:
            shape = [3, height, width] if self.channels_first else [height, width, 3]
            self._observation_space = spaces.Box(
                low=0, high=255, shape=shape, dtype=np.uint8
            )
        else:
            self._observation_space = _spec_to_box(
                self._env.observation_spec().values()
            )
            
        self._state_space = _spec_to_box(
                self._env.observation_spec().values()
        )
        
        self.current_state = None

        # set seed
        self.seed(seed=task_kwargs.get('random', 1))

    def __delete__(self):
        self.camera.stop()

    @property
    def observation_space(self):
        return self._observation_space

    @property
    def state_space(self):
        return self._state_space

    @property
    def action_space(self):
        return self._norm_action_space

    def step(self,a):
        for _ in range(self._frame_skip):
            # time_step = self._env.step(action)
            # reward += time_step.reward or 0
            if control=="torque":
                self.blue.set_joint_torques(a[:-1])
                self.blue.command_gripper(a[-1], 20.0, wait=True)
            else:
                self.blue.set_joint_positions(a[:-1], duration=0.1)
                self.blue.command_gripper(a[-1], 20.0, wait=True)
        obs = self._get_obs()
        reward = 0
        done = False
        return obs, reward, done, None

    def reset(self):
        init_pos = np.array([0, -2.31, 1.57, -0.75, -1.57, 0, 0])
        self.blue.set_joint_positions(init_pos, duration=0.0)
        self.blue.calibrate_gripper()
        self.blue.command_gripper(0.0, 20.0, wait=False)
        obs = self._get_obs()
        return obs

    def _get_obs(self):
        if self._from_pixels:
            obs = self.render()
            if self._channels_first:
                obs = obs.transpose(2, 0, 1).copy()
        else:
            obs = get_robot_joints()
        return obs

    def render(self):
        return self.camera()

    @property
    def get_qpos(self):
        joint = self.blue.get_joint_positions()
        gripper = self.blue.get_gripper_position()
        return np.concatenate((joint,gripper),axis=0)

    @property
    def get_qvel(self):
        joint = self.blue.get_joint_velocities()
        gripper = 0
        return np.concatenate((joint,gripper),axis=0)

    def get_robot_joints(self):
        return np.concatenate([
            self.get_qpos,
            self.get_qvel,
        ])

# blue.calibrate_gripper()
# blue.cancel_gripper_command()
# blue.command_gripper(2.0, 3.0)
# print("cartesian_pose: ",blue.get_cartesian_pose())
# print("gripper_effort: ",blue.get_gripper_effort())
# print("gripper_position: ",blue.get_gripper_position())
# print("joint_position: ",blue.get_joint_positions())
# print("joint_torques: ",blue.get_joint_torques())

# while True:
#     pass% 



class StreamVideo(object):
    def __init__(self, name="VideoStream", height=100, width=100, camera_id=0):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.front_stream = cv2.VideoCapture(camera_id)
        (self.front_grabbed, self.front_frame) = self.front_stream.read()
        #self.front_frame = self.top_frame = np.empty((256,256,3), dtype=np.uint8)

        # initialize the thread name
        self.name = name

        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

        self.cropx = 480
        self.cropy = 480
        self.size = height
        
        self.lock = Lock()


    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, name=self.name, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # otherwise, read the next frame from the stream
            (front_grabbed, front_frame) = self.front_stream.read()

            front_frame = cv2.cvtColor(front_frame, cv2.COLOR_BGR2RGB)

            front_frame = self.crop_center(front_frame)
            self.lock.acquire() 
            self.front_frame = self.resize(front_frame)
            self.lock.release()

    def read(self):
        # return the frame most recently read
        return self.front_frame
    
    def __call__(self):
        return self.front_frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
    
    def crop_center(self, img):
        y,x, _ = img.shape
        startx = x//2-(self.cropx//2)
        starty = y//2-(self.cropy//2)    
        return img[starty:starty+self.cropy,startx:startx+self.cropx]
    
    def resize(self, img):
        return cv2.resize(img, (self.size, self.size), interpolation=cv2.INTER_CUBIC)

if __name__ = "__main__":
    import matplotlib.pyplot as plt
    b = BlueRobotEnv()
    obs = b.reset()
    plt.imshow(obs)
    plt.show()
    a = np.zeros(7)
    obs = b.step(a)
    plt.imshow(obs)
    plt.show()