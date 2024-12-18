import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

import os 
import sys 
cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  # include pybind module

# Load the URDF file
robot = RobotWrapper.BuildFromURDF(cwd + "/robot_model/crab/crab.urdf", [cwd + "/robot_model/crab"])

# Access the model
model = robot.model

# Print joint names and their indices
for i, name in enumerate(robot.model.names):
    print(f"Joint index: {i}, Joint name: {name}")

# Print link (frame) names and their indices
for i, frame in enumerate(robot.model.frames):
    print(f"Frame index: {i}, Frame name: {frame.name}")

