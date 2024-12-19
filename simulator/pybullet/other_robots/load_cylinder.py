import pybullet as pb
import pybullet_data
import os

# Connect to PyBullet and load the GUI
pb.connect(pb.GUI)

# Set the search path to find URDF files
pb.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the URDF files
# cwd = os.getcwd()
# cylinder_robot = pb.loadURDF(
#     cwd + "/robot_model/cylinder.urdf",
#     [2.5, 5.0, 2.5],
#     [0, 0.707, 0, 0.707],
#     useFixedBase=False,
# )
# ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf", useFixedBase=1)

# Load a simple plane and a cube
plane = pb.loadURDF("plane.urdf")
cube = pb.loadURDF("r2d2.urdf", [0, 0, 1])

# Enable rendering
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

# Set the camera view
pb.resetDebugVisualizerCamera(
    cameraDistance=10,  # Distance from the target
    cameraYaw=45,       # Yaw angle in degrees
    cameraPitch=-30,    # Pitch angle in degrees
    cameraTargetPosition=[0, 0, 0]  # Target position
)

# Run the simulation
while True:
    pb.stepSimulation()