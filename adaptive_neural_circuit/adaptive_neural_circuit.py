"""
Adaptive neural circuit
"""


import os
import sys
sys.path.append(os.getcwd())

import time
import random
import numpy as np
import matplotlib.pyplot as plt


class ICO:
    def __init__(self, lr:float=1e-2):
        self.w = 0.1 * np.random.randn(1)
        self.lr = lr
        self.val = 0.0

    def forward(self, x1, x2):
        self.val = self.val + self.w * x1 + x2

    def update_w(self, x):
        self.w = self.w + self.lr * x


if __name__ == '__main__':
    from tqdm import tqdm

    br = ICO()
    bl = ICO()
    w = ICO()

    r = 300     # Straight line distance between robot and goal [cm]
    v_l = 0     # Linear velocity of left wheel [cm/s]
    v_r = 0     # Linear velocity of right wheel [cm/s]
    l = 16      # Width of robot in [cm]
    
    collision_thresh = 20   # If value below, the reflex signal is triggered
    detection_thresh = 40   # If value below, the obstacle is considered

    dists_moved = list()

    for i in tqdm(range(1, 50)):
        start_time = time.time()

        dist_moved = 0
        oL, oR = list(), list()
        vl, vr = list(), list()

        goal = np.random.randn(2) * 10   # Goal position [x, y]
        obstacles = np.random.randn(2,2) * 5    # Obstacles positions [x, y]

        robot_pose = np.random.randn(3)    # Robot pose [x, y, theta]
        robot_omega = (v_r - v_l) / l      # Robot angular velocity
        
        sensor = [robot_pose[0], robot_pose[1], collision_thresh, detection_thresh]
        reflex = 0

        prev_closest_obs = np.array([0.0, 0.0, 0.0])
        for j in range(1, 1000):
            last_robot_xy = np.array([robot_pose[0], robot_pose[1]])

            # Determine the robot position relative to the goal
            goal_angle = np.arctan2(goal[1] - robot_pose[1], goal[0] - robot_pose[0])

            # Determine the distance from the robot to the goal
            goal_dist = np.linalg.norm(goal-robot_pose[:2])

            # Determine the distance to the obstacles and find all obstalces within threshold
            obs_collision, obs_detection = list(), list()
            for obs in obstacles:
                obs_dist = np.linalg.norm(obs-robot_pose[:2])
                if obs_dist <= sensor[2]: obs_collision.append((obs[0], obs[1], obs_dist))
                elif obs_dist <= sensor[3]: obs_detection.append((obs[0], obs[1], obs_dist))

            obs_collision = np.array(obs_collision)
            obs_detection = np.array(obs_detection)

            if len(obs_collision):
                # Collision obstacles
                closest_obs = min(obs_collision, key=lambda x: x[2])
                reflex += 1

                theta = np.arctan2(closest_obs[1] - robot_pose[1],
                                   closest_obs[0] - robot_pose[0] - robot_pose[2])
                
                if theta < 0 and theta > np.deg2rad(-90):   # Right side of robot
                    rr, v_l, v_r = 1, 0.1, 4.0

                    x = ((closest_obs[2]-prev_closest_obs[2]))/collision_thresh
                    br.update_w(x)

                    x1 = goal_angle
                    x2 = closest_obs[2]/collision_thresh
                    br.forward(x1, x2)
                elif theta > 0 and theta < np.deg2rad(90):  # Left side of robot
                    rl, v_l, v_r = 1, 4.0, 0.1

                    x = ((closest_obs[2]-prev_closest_obs[2]))/collision_thresh
                    bl.update_w(x)

                    x1 = goal_angle
                    x2 = closest_obs[2]/collision_thresh
                    bl.forward(x1, x2)

                x = (min(obs_detection, key=lambda x: x[2])[2]/sensor[3]) * (closest_obs[2]-prev_closest_obs[2])/sensor[2]
                w.update_w(x)
            elif len(obs_detection):
                # Detected obstacles - no reflex
                closest_obs = min(obs_detection, key=lambda x: x[2])

                x1 = closest_obs[2] / sensor[3]
                x2 = closest_obs[2] / sensor[2]
                w.forward()

                theta = np.arctan2(closest_obs[1] - robot_pose[1],
                                   closest_obs[0] - robot_pose[0] - robot_pose[2])

                if theta < 0 and theta > np.deg2rad(-90):   # Right side of robot
                    v_l = 4 / (1 + br.val * np.exp(-1 * goal_angle))
                    v_r = 4 / (1 + bl.val * np.exp(-1 * goal_angle)) + w.val
                elif theta > 0 and theta < np.deg2rad(90): # Left side of robot
                    v_l = 4 / (1 + br.val * np.exp(-1 * goal_angle)) + w.val
                    v_r = 4 / (1 + bl.val * np.exp(-1 * goal_angle))
                else:
                    v_l = 4 / (1 + br.val * np.exp(-1 * goal_angle))
                    v_r = 4 / (1 + bl.val * np.exp(-1 * goal_angle))

            # Update robots pose based on new motor velocities
            if v_l != v_r:
                R = (l/2) * (v_r + v_l) / (v_r - v_l)
                robot_omega = (v_r - v_l) / l
                icc = [
                    robot_pose[0] - (R * np.sin(robot_pose[2])),
                    robot_pose[1] + (R * np.cos(robot_pose[2]))
                ]
                robot_pose = [
                    
                ]

        
        if reflex == 0 and goal_dist == 0:
            print(f'Robot reached the goal!')
            break
