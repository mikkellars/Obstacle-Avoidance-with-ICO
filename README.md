# Project in Artificial Intelligence

This repository contains the code for the 5 ETCS course Project in Artificial Intelligence.

Programming a robot the traditional way to solve problems is a difficult task. Making the robot learn to solve a problems is a more robust way as the robot can adapt to its environment. In this project, a system for avoiding obstacle and following a target for differential drive mobile robot is developed. The system is based on input correlation learning which is an unsupervised machine learning method, making the system capable of learning its environment it is deployed in. The system was deployed on a small differential drive mobile robot containing a mono camera, a Raspberry Pi 4, a Matrix Voice and an Edge TPU.

The mobile robot learned to navigate in a indoor environment at the Technical Faculty at Southern University of Denmark, here it was able to locate a person, i.e. a target, by the use of computer vision. Furthermore, it learned to avoid static obstacles and drive around them in a smoothly manner.

This project shows, how a simple system can learn to smoothly navigate in an indoor environment. The system could be used create a more interaction trash disposal option, thus, instead of having a static trashcan a mobile robot with a trashcan could be setup. Further applications could also be added, e.g. a robot arm to collect forgotten trash.

## Obstacle avoidance

| Iteration 1 | Iteration 2 | Iteration 3 |
|:-----------:|:-----------:|:-----------:|
| ![](data_analyse/assets/test_box_left/01.gif) | ![](data_analyse/assets/test_box_left/02.gif) | ![](data_analyse/assets/test_box_left/03.gif) |
| ![](data_analyse/assets/test_box_left/01.png) | ![](data_analyse/assets/test_box_left/02.png) | ![](data_analyse/assets/test_box_left/03.png) |

| Obstacle Collisions | Obstacle Avoidance ICO | Trajectory |
|:-------------------:|:----------------------:|:----------:|
| ![](data_analyse/assets/test_box_left/obs_cols.png) | ![](data_analyse/assets/test_box_left/obs_icos.png) | ![](data_analyse/assets/test_box_left/trajectory.png) 

| Iteration 1 | Iteration 2 | Iteration 3 |
|:-----------:|:-----------:|:-----------:|
| ![](data_analyse/assets/test_wall_left/01.gif) | ![](data_analyse/assets/test_wall_left/02.gif) | ![](data_analyse/assets/test_wall_left/03.gif) |
| ![](data_analyse/assets/test_wall_left/01.png) | ![](data_analyse/assets/test_wall_left/02.png) | ![](data_analyse/assets/test_wall_left/03.png) |

| Obstacle Collisions | Obstacle Avoidance ICO | Trajectory |
|:-------------------:|:----------------------:|:----------:|
| ![](data_analyse/assets/test_wall_left/obs_col.png) | ![](data_analyse/assets/test_wall_left/obs_icos.png) | ![](data_analyse/assets/test_wall_left/trajectory.png) 

## Human following

| Iteration 1 | Iteration 2 | Iteration 3 |
|:-----------:|:-----------:|:-----------:|
| ![](data_analyse/assets/test_human_left/01.gif) | ![](data_analyse/assets/test_human_left/02.gif) | ![](data_analyse/assets/test_human_left/03.gif) |
| ![](data_analyse/assets/test_human_left/01.png) | ![](data_analyse/assets/test_human_left/02.png) | ![](data_analyse/assets/test_human_left/03.png) |

| Reflective Signal | Following ICO | Trajectory |
|:-------------------:|:----------------------:|:----------:|
| ![](data_analyse/assets/test_human_left/ico_human_col.png) | ![](data_analyse/assets/test_human_left/human_mc.png) | ![](data_analyse/assets/test_human_left/trajectory.png)

## Human following and obstacle avoidance

| Iteration 1 | Iteration 2 | Iteration 3 |
|:-----------:|:-----------:|:-----------:|
| ![](data_analyse/assets/test_human_box/01.gif) | ![](data_analyse/assets/test_human_box/02.gif) | ![](data_analyse/assets/test_human_box/03.gif) |
| ![](data_analyse/assets/test_human_box/01.png) | ![](data_analyse/assets/test_human_box/02.png) | ![](data_analyse/assets/test_human_box/03.png) |

| Trajectory |
|:----------:|
| ![](data_analyse/assets/test_human_box/trajectory.png)

<!--
| Reflective Signal | Following ICO |
|:-------------------:|:----------------------:|
| ![](data_analyse/assets/test_human_box/human_col.png) | ![](data_analyse/assets/test_human_box/human_mc.png) |

| Obstacle Collisions | Obstacle Avoidance ICO |
|:-------------------:|:----------------------:|
| ![](data_analyse/assets/test_human_box/obs_cols.png) | ![](data_analyse/assets/test_human_box/obs_icos.png) |
-->
