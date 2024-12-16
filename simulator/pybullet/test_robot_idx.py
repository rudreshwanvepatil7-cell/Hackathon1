import pybullet as pb
import pybullet_data
import os
import sys
import numpy as np

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  # include pybind module

# Connect to PyBullet
pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the robot model
robot = pb.loadURDF(
    cwd + "/robot_model/crab/crab.urdf",
    [0.0, 0.0, 2.0],
    [0, 0, 0, 1],
    useFixedBase=False,
)

# # Print link names and their indices
# num_joints = pb.getNumJoints(robot)
# for i in range(num_joints):
#     link_name = pb.getJointInfo(robot, i)[12].decode('utf-8')
#     print(f"Link index: {i}, Link name: {link_name}") 
    
# Print joint names and their indices
print("\nJoint names and their indices:")
for i in range(num_joints):
    joint_name = pb.getJointInfo(robot, i)[1].decode('utf-8')
    print(f"Joint index: {i}, Joint name: {joint_name}")


# # Define the Go2LinkIdx class with updated indices
# class Go2LinkIdx(object):
#     base = -1
#     Head_upper = 0
#     Head_lower = 1
#     FL_hip = 2
#     FL_thigh = 3
#     FL_calf = 4
#     FL_calflower = 5
#     FL_calflower1 = 6
#     FL_foot = 7
#     FR_hip = 8
#     FR_thigh = 9
#     FR_calf = 10
#     FR_calflower = 11
#     FR_calflower1 = 12
#     FR_foot = 13
#     RL_hip = 14
#     RL_thigh = 15
#     RL_calf = 16
#     RL_calflower = 17
#     RL_calflower1 = 18
#     RL_foot = 19
#     RR_hip = 20
#     RR_thigh = 21
#     RR_calf = 22
#     RR_calflower = 23
#     RR_calflower1 = 24
#     RR_foot = 25
#     imu = 26
#     radar = 27

# # Existing functionality from crab_robot_main.py
# cwd = os.getcwd()
# sys.path.append(cwd)
# sys.path.append(cwd + "/build/lib")  # include pybind module

# from config.crab.sim.pybullet.wbic.pybullet_params import *
# from util.python_utils import pybullet_util
# from util.python_utils import util
# from util.python_utils import liegroup

# import signal
# import shutil
# import cv2

# import go2_interface_py 

# if Config.MEASURE_COMPUTATION_TIME:
#     from pytictoc import TicToc

# def get_sensor_data_from_pybullet(robot):
#     # follow pinocchio robotsystem urdf reading convention
#     joint_pos, joint_vel = np.zeros(12), np.zeros(12)

#     imu_frame_quat = np.array(pb.getLinkState(robot, crab_link_idx.imu, 1, 1)[1])

#     joint_vel[9] = pb.getJointState(robot, crab_joint_idx.RR_hip_joint)[1]
#     joint_vel[10] = pb.getJointState(robot, crab_joint_idx.RR_thigh_joint)[1]
#     joint_vel[11] = pb.getJointState(robot, crab_joint_idx.RR_calf_joint)[1] 

#     # normal force measured on each foot
#     FL_normal_force = 0
#     contacts = pb.getContactPoints(bodyA=robot, linkIndexA=crab_link_idx.FL_foot)
#     for contact in contacts:
#         # add z-component on all points of contact
#         FL_normal_force += contact[9]

#     FR_normal_force = 0
#     contacts = pb.getContactPoints(bodyA=robot, linkIndexA=crab_link_idx.FR_foot)
#     for contact in contacts:
#         # add z-component on all points of contact
#         FR_normal_force += contact[9]

#     RL_normal_force = 0
#     contacts = pb.getContactPoints(bodyA=robot, linkIndexA=crab_link_idx.RL_foot)
#     for contact in contacts:
#         # add z-component on all points of contact
#         RL_normal_force += contact[9]

#     RR_normal_force = 0
#     contacts = pb.getContactPoints(bodyA=robot, linkIndexA=crab_link_idx.RR_foot)
#     for contact in contacts:
#         # add z-component on all points of contact
#         RR_normal_force += contact[9]

# # Example usage
# get_sensor_data_from_pybullet(robot)

