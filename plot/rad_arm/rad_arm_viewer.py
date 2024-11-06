import os
import sys
import pinocchio as pin 
import time

cwd = os.getcwd()
sys.path.append(cwd)

# Robot model libraries
from pinocchio.visualize import MeshcatVisualizer


# Create Robot for Meshcat Visualization
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    cwd + "/robot_model/rad_arm/rad_arm.urdf",
    cwd + "/robot_model/rad_arm/",
    pin.JointModelFreeFlyer(),
)
viz = MeshcatVisualizer(model, collision_model, visual_model)
try:
    viz.initViewer(open=True)
    # viz.viewer.wait()
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)
viz.loadViewerModel(rootNodeName="rad_arm")
vis_q = pin.neutral(model)
viz.display(vis_q)

time.sleep(5)

print("done")