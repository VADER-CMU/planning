# VADER Planning

## Attribution

This repository belongs to Carnegie Mellon University, Masters of Science - Robotic Systems Development (MRSD) Team E - VADER

Team Members: Tom Gao, Abhishek Mathur, Rohit Satishkumar, Kshitij Bhat, Keerthi GV 

Special thanks to the authors at UFactory Inc. of the original xarm_ros repository. Some of the work in this repository were taken in reference to their work. Their original work is referenced from https://github.com/xArm-Developer/xarm_ros/tree/master.

First Revision: February 2025

## Introduction and Overview

This is the VADER Motion Planner repository, which stores all planning related code for the single-arm and bimanual configurations. The single-arm and bimanual planners are called from the state machines with specific service files, and manage the trajectory generation and execution of the arm planning groups.

The choice of planner backend algorithms are specified in `xarm7_moveit_config` (for single arm planner) and `uf_robot_moveit_config` (for dual arm). At the moment, the default backend is RRTStar, and for short translation movements with no collision objects, cartesian planning is used.

## Usage

Clone this repository into your workspace and build it. The files in the HRI repository will launch the `vader_SVD_dual_sim.launch` and `vader_SVD_single_realHW.launch` files as appropriate and boot up the planner.