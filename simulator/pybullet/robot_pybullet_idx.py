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

num_joints = pb.getNumJoints(robot)

# Print joint names and their indices
for i in range(num_joints):
    joint_info = pb.getJointInfo(robot, i)
    print(f"Joint index: {i}, Joint name: {joint_info[1].decode('utf-8')}")

# Print link names and their indices
for i in range(num_joints):
    link_state = pb.getLinkState(robot, i)
    print(f"Link index: {i}, Link name: {link_state[12].decode('utf-8')}")



