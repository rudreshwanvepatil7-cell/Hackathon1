import pybullet as pb
import pybullet_data
import os

## connect pybullet sim server
pb.connect(pb.GUI)
pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

pb.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the URDF file
cwd = os.getcwd()
robot = pb.loadURDF(cwd + "/robot_model/crab/crab.urdf", [0.0, 0.0, 2.0], [0, 0, 0, 1], useFixedBase=False)

# Load the skybox texture
skybox_texture = pb.loadTexture("simulator/pybullet/starmap_2020_4k_print.png")

# Apply the texture to the environment
pb.changeVisualShape(-1, -1, textureUniqueId=skybox_texture)

# Run the simulation
count = 0 
while True:
    pb.stepSimulation()  # step simulation
    count += 1