# Mobile Robotic Manipulation
My final project for Robotic Manipulation (ME-449), a course at Northwestern University.

<p align="center">
  <img align="center" src="https://github.com/seanmorton2023/ME449/blob/master/HW/Project/media/demo1_class_task.gif" width="90%">
</p>


## Table of Contents

- [Project Description](#project-description)
- [Robot Specification] (#robot-specification)
- [Results](#results)


## Project Description

In this project, I wrote software that plans a trajectory for the end-effector of the Kuka youBot mobile manipulator (a mobile base with four mecanum wheels and a 5R robot arm). The software performs odometry as the chassis moves, and performs feedback control to drive the youBot to pick up a block at a specified location, carry it to a desired location, and put it down.

The output of the Python script is a comma-separated values (csv) text file that specifies the configurations of the chassis and the arm, the angles of the four wheels, and the state of the gripper (open or closed) as a function of time. This specification of the position-controlled youBot was then imported into the CoppeliaSim simulator to demonstrate the result of the task.

## Robot Specification

The Kuka YouBot is described by the frames and link lengths shown below:

<p align="center">
  <img align="center" src="https://github.com/seanmorton2023/ME449/blob/master/HW/Project/media/youbot_full_diagram.png" width="90%">
</p>

A closer look at the mecanum wheels at the base of the mobile manipulator is shown below:

<p align="center">
  <img align="center" src="https://github.com/seanmorton2023/ME449/blob/master/HW/Project/media/youbot_diagram.png" width="90%">
</p>



## Results

Demonstrations of the final software are shown below.
________

Demo #1: Simulation with the mobile manipulator, showing the primary motion goal of the robot.

<p align="center">
  <img align="center" src="https://github.com/seanmorton2023/ME449/blob/master/HW/Project/media/demo1_class_task.gif" width="90%">
</p>
________

Demo #2: Simulation with the mobile manipulator, showing an extra task I completed, in which joint limits were imposed on the motion of the arm.

<p align="center">
  <img align="center" src="https://github.com/seanmorton2023/ME449/blob/master/HW/Project/media/demo2_extra_joint_limits.gif" width="90%">
</p>
