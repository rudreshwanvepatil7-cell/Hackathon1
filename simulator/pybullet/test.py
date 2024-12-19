import pybullet as pb
import pybullet_data
import os


## connect pybullet sim server
pb.connect(pb.GUI)
pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
## robot spawn & initial kinematics and dynamics setting
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

pb.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the URDF file
cwd = os.getcwd()

robot = pb.loadURDF(cwd + "/robot_model/crab/crab.urdf", [0.0, 0.0, 2.0], [0, 0, 0, 1], useFixedBase=False)

# Run the simulation
count = 0 
while True:
    pb.stepSimulation()  # step simulation
    count += 1