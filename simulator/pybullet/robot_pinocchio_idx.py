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

# Print the names and indices of the joints
print("Joint names and their indices:")
for i, name in enumerate(model.names):
    if i > 1:  # Skip the first two entries which are the universe and the root joint
        print(f"Joint index: {i}, Joint name: {name}")

# Print the names and indices of the links (frames)
print("\nLink (frame) names and their indices:")
for i, frame in enumerate(model.frames):
    print(f"Frame index: {i}, Frame name: {frame.name}")