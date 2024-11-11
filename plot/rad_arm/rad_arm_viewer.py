import os
import sys
import pinocchio as pin 
import time

# Get the current working directory
cwd = os.getcwd()

# Append the current working directory to the system path
sys.path.append(cwd)

# Import the MeshcatVisualizer from pinocchio.visualize
from pinocchio.visualize import MeshcatVisualizer

# Create Robot for Meshcat Visualization
# Build the robot models from the URDF file
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    cwd + "/robot_model/crab/crab.urdf",  # Path to the URDF file
    cwd + "/robot_model/crab/",  # Path to the robot model directory
    pin.JointModelFreeFlyer(),  # Use a free-flyer joint model
)

# Initialize the Meshcat visualizer with the robot models
viz = MeshcatVisualizer(model, collision_model, visual_model)

try:
    # Try to initialize the viewer
    viz.initViewer(open=True)
    # viz.viewer.wait()  # Uncomment if you want the viewer to wait

except ImportError as err:
    
    # Handle the case where the Meshcat viewer is not installed
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

# Load the robot model into the viewer
viz.loadViewerModel(rootNodeName="rad_arm")

# Get the neutral configuration of the robot model
vis_q = pin.neutral(model)

# Display the neutral configuration in the viewer
viz.display(vis_q)

# Sleep for 5 seconds to allow the viewer to display the model
time.sleep(5)

# Print done when finished
print("done")