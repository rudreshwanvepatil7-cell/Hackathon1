import pybullet as pb
import os
import sys
import numpy as np
import signal
import shutil
import cv2

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  # include pybind module

from config.crab.sim.pybullet.ihwbc.pybullet_params import *
from util.python_utils import pybullet_util
from util.python_utils import util
from util.python_utils import liegroup

import signal
import shutil
import cv2

# import go2_interface_py
import crab_interface_py

# moved some functions to simulator.pybullet.crab_pybullet_fns
from simulator.pybullet.crab_pybullet_fns import *

# ----------------------------------
# main simulation loop
# ----------------------------------

if __name__ == "__main__":
    ## connect pybullet sim server
    pb.connect(pb.GUI)
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

    base_pos = [0., 0., 0.]
    # base_pos = Config.INITIAL_BASE_JOINT_POS
    pb.resetDebugVisualizerCamera(
        cameraDistance=10,
        cameraYaw=120,
        cameraPitch=-15,
        cameraTargetPosition= np.array([0.5, 0.3, 1.5]),
    ) 

    ## sim physics setting
    pb.setPhysicsEngineParameter(
        fixedTimeStep=Config.CONTROLLER_DT, numSubSteps=Config.N_SUBSTEP
    )
    pb.setGravity(0, 0, 0)

    ## robot spawn & initial kinematics and dynamics setting
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)

    ## LOAD ROBOT
    robot = pb.loadURDF(
        cwd + "/robot_model/crab/crab.urdf",
        base_pos,
        [0, 0, 0, 1],
        useFixedBase=False,
    )

    # Set the initial linear velocity
    initial_linear_velocity = [0.0, 0.0, 0.0]  # Example: zero initial velocity
    initial_angular_velocity = [0.0, 0.0, 0.0]  # No initial angular velocity

    pb.resetBaseVelocity(
        robot,
        linearVelocity=initial_linear_velocity,
        angularVelocity=initial_angular_velocity,
    )
    
    red_ball = pb.loadURDF( 
        cwd + "/robot_model/crab/red_ball.urdf", 
        [6.0, 0.0, 0.0], 
        [0, 0, 0, 1], 
        useFixedBase=True 
    ) 
    
    green_ball = pb.loadURDF( 
        cwd + "/robot_model/crab/green_ball.urdf", 
        [0.0, 6.0, 0.0], 
        [0, 0, 0, 1], 
        useFixedBase=True 
    ) 
    
    blue_ball = pb.loadURDF( 
        cwd + "/robot_model/crab/blue_ball.urdf", 
        [0.0, 0.0, 6.0], 
        [0, 0, 0, 1], 
        useFixedBase=True 
    ) 

    # ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf", useFixedBase=True)

    # Create a large plane to simulate a background
    plane_shape  = pb.createCollisionShape(pb.GEOM_PLANE)
    plane_visual = pb.createVisualShape(pb.GEOM_PLANE, rgbaColor=[0, 0, 0, 1])
    plane = pb.createMultiBody(0, plane_shape, plane_visual)

    # Position the plane far away to act as a background
    pb.resetBasePositionAndOrientation(plane, [0, 0, -100], [0, 0, 0, 1])

    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)

    (
        n_q,
        n_v,
        n_a,
        joint_id_dict,
        link_id_dict,
        pos_basejoint_to_basecom,
        rot_basejoint_to_basecom,
    ) = pybullet_util.get_robot_config(
        robot,
        Config.INITIAL_BASE_JOINT_POS,
        Config.INITIAL_BASE_JOINT_QUAT,
        Config.PRINT_ROBOT_INFO,
    )
    
    # robot initial config setting
    # set_init_config_pybullet_robot(robot)
    set_0_config_robot(robot) 

    # robot joint and link dynamics setting
    pybullet_util.set_joint_friction(robot, joint_id_dict, 0)
    pybullet_util.set_link_damping(robot, link_id_dict, 0.0, 0.0)

    # default robot kinematics information
    base_com_pos, base_com_quat = pb.getBasePositionAndOrientation(robot)
    rot_world_basecom = util.quat_to_rot(np.array(base_com_quat))
    rot_world_basejoint = util.quat_to_rot(np.array(Config.INITIAL_BASE_JOINT_QUAT))

    pos_basejoint_to_basecom = np.dot(
        rot_world_basejoint.transpose(),
        base_com_pos - np.array(Config.INITIAL_BASE_JOINT_POS),
    )
    rot_basejoint_to_basecom = np.dot(
        rot_world_basejoint.transpose(), rot_world_basecom
    )

    # Run Simulation
    dt = Config.CONTROLLER_DT
    count = 0
    jpg_count = 0

    ## simulation options
    if Config.MEASURE_COMPUTATION_TIME:
        timer = TicToc()
        compuation_cal_list = []

    if Config.VIDEO_RECORD:
        video_dir = "video/crab"
        if os.path.exists(video_dir):
            shutil.rmtree(video_dir)
        os.makedirs(video_dir)

    ## for dvel quantity
    previous_torso_velocity = np.array([0.0, 0.0, 0.0]) 
    
    # create arrows 
    x_arrow, y_arrow, z_arrow, z_neg_arrow = update_arrows( base_com_pos, rot_world_basecom ) 
    
    # ---------------------------------- 
    # SIM LOOP 
    # ---------------------------------- 
    
    # sequences = generate_momentum_test_sequence() 
    
    # for command, duration in sequences:
        
    #     steps = int(duration * 1/dt)  # Assuming 240Hz simulation
        
    #     for _ in range(steps):
        
    #         apply_control_input_to_pybullet(robot, command)
    #         pb.stepSimulation()
            
    #         # Record orientation for analysis
    #         pos, ori = pb.getBasePositionAndOrientation(robot)
    #         rot = np.array(pb.getMatrixFromQuaternion(ori)).reshape(3,3)
    #         z_axis = rot[:,2]
    #         print(f"Current Z-axis orientation: {z_axis}")

    while True: 
        
        # ---------------------------------- 
        # compute distance from end effectors to cylinder 
        # ---------------------------------- 
        
        x_arrow, y_arrow, z_arrow, z_neg_arrow = update_arrows( base_com_pos, rot_world_basecom, x_arrow, y_arrow, z_arrow, z_neg_arrow )

        # ----------------------------------
        # compute control command
        # ----------------------------------

        if Config.MEASURE_COMPUTATION_TIME:
            timer.tic()

        if Config.MEASURE_COMPUTATION_TIME:
            comp_time = timer.tocvalue()
            compuation_cal_list.append(comp_time)
        
        
        # Get current angles
        left_state  = pb.getJointState(robot, crab_joint_idx.front_left__cluster_1_roll)
        right_state = pb.getJointState(robot, crab_joint_idx.front_right__cluster_1_roll)
        
        # print_joint_state(robot, crab_joint_idx.front_left__cluster_1_pitch)
        
        left_angle  = np.degrees(left_state[0])
        right_angle = np.degrees(right_state[0]) 
        
        trq = 0 
        # if abs(left_angle) < 1.57:    
        #     print(f"left angle = {left_angle}")
        #     trq = 0.1

        # apply command to pybullet robot: 
            # cluster_1_roll, cluster_1_pitch, 
            # cluster_2_roll, cluster_2_pitch, 
            # cluster_3_roll, cluster_3_pitch, cluster_3_wrist 
        values = [
            0, -trq, 0, 0, 0, 0, 0, 
            0, -trq, 0, 0, 0, 0, 0, 
            0, -trq, 0, 0, 0, 0, 0, 
            0, -trq, 0, 0, 0, 0, 0 
        ] 
        
        ones_array = np.ones(28) 

        # Convert the list to a NumPy array
        rpc_trq_command = np.array(values)
        # rpc_trq_command = ones_array 
        
        apply_control_input_to_pybullet(robot, rpc_trq_command) 
        
        # use rpc_joint_pos_command and vel_command in PD controller 
        # local impedance controller 
        
        # save current torso velocity for next iteration
        previous_torso_velocity = pybullet_util.get_link_vel(
            robot, crab_link_idx.base_link
        )[3:6] 

        # ----------------------------------
        # Step simulation 
        # ----------------------------------

        pb.stepSimulation()  # step simulation

        count += 1
