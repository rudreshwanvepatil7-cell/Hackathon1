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
    initial_linear_velocity  = [0.0, 0.0, 0.0]  # Example: zero initial velocity
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
    
    target_pos, _ = pb.getBasePositionAndOrientation(green_ball)
    print(f"pybullet target_pos = {target_pos}")

    # ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf", useFixedBase=True)

    # Create a large plane to simulate a background
    plane_shape = pb.createCollisionShape(pb.GEOM_PLANE)
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

    # TODO: pnc interface, sensor_data, command class
    rpc_crab_interface = crab_interface_py.CrabInterface()
    rpc_crab_sensor_data = crab_interface_py.CrabSensorData()
    rpc_crab_command = crab_interface_py.CrabCommand()

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
    
    x_arrow, y_arrow, z_arrow, z_neg_arrow = update_arrows( base_com_pos, rot_world_basecom ) 
    
    # # initialize PID controller 
    # pid = PIDController(kp = 1.0, ki = 0.0, kd = 0.1)

    while True:
        # ----------------------------------
        # Moving Camera Setting
        # ----------------------------------

        # base_pos, base_ori = pb.getBasePositionAndOrientation(robot)
        # pb.resetDebugVisualizerCamera(cameraDistance=12.5,
        #                               cameraYaw=120,
        #                               cameraPitch=-45,
        #                               cameraTargetPosition=base_pos +
        #                               np.array([0.5, 0.3, -base_pos[2] + 1.5]))

        # ----------------------------------
        # Debugging
        # ----------------------------------

        base_com_pos, base_com_quat = pb.getBasePositionAndOrientation(robot)
        rot_world_basecom = util.quat_to_rot(base_com_quat)
        rot_world_basejoint = np.dot(
            rot_world_basecom, rot_basejoint_to_basecom.transpose()
        )
        base_joint_pos = base_com_pos - np.dot(
            rot_world_basejoint, pos_basejoint_to_basecom
        )
        base_joint_quat = util.rot_to_quat(rot_world_basejoint)

        base_com_lin_vel, base_com_ang_vel = pb.getBaseVelocity(robot)
        trans_joint_com = liegroup.RpToTrans(
            rot_basejoint_to_basecom, pos_basejoint_to_basecom
        )
        adjoint_joint_com = liegroup.Adjoint(trans_joint_com)
        twist_basecom_in_world = np.zeros(6)
        twist_basecom_in_world[0:3] = base_com_ang_vel
        twist_basecom_in_world[3:6] = base_com_lin_vel
        augrot_basecom_world = np.zeros((6, 6))
        augrot_basecom_world[0:3, 0:3] = rot_world_basecom.transpose()
        augrot_basecom_world[3:6, 3:6] = rot_world_basecom.transpose()
        twist_basecom_in_basecom = np.dot(augrot_basecom_world, twist_basecom_in_world)
        twist_basejoint_in_basejoint = np.dot(
            adjoint_joint_com, twist_basecom_in_basecom
        )
        augrot_world_basejoint = np.zeros((6, 6))
        augrot_world_basejoint[0:3, 0:3] = rot_world_basejoint
        augrot_world_basejoint[3:6, 3:6] = rot_world_basejoint
        twist_basejoint_in_world = np.dot(
            augrot_world_basejoint, twist_basejoint_in_basejoint
        )
        base_joint_ang_vel = twist_basejoint_in_world[0:3]
        base_joint_lin_vel = twist_basejoint_in_world[3:6]

        # pdb.set_trace()

        # pass debugged data to rpc interface (for ground truth estimation)
        rpc_crab_sensor_data.base_joint_pos_ = base_joint_pos
        rpc_crab_sensor_data.base_joint_quat_ = base_joint_quat
        rpc_crab_sensor_data.base_joint_lin_vel_ = base_joint_lin_vel
        rpc_crab_sensor_data.base_joint_ang_vel_ = base_joint_ang_vel

        # ----------------------------------
        # Get Keyboard Event
        # ----------------------------------

        keys = pb.getKeyboardEvents()
        if pybullet_util.is_key_triggered(keys, "1"):
            pass

        # ----------------------------------
        # Get Sensor Data
        # ----------------------------------
        
        rpc_crab_sensor_data = get_assign_sensor_data(robot, rpc_crab_sensor_data, dt, previous_torso_velocity) 
        
        # ---------------------------------- 
        # compute distance from end effectors to cylinder 
        # ---------------------------------- 
        
        x_arrow, y_arrow, z_arrow, z_neg_arrow = update_arrows( base_com_pos, rot_world_basecom, x_arrow, y_arrow, z_arrow, z_neg_arrow )
        
        # # Get the position of each end effector
        # lfoot_pos = pb.getLinkState(robot, crab_link_idx.back_left__foot_link)[0]
        # rfoot_pos = pb.getLinkState(robot, crab_link_idx.back_right__foot_link)[0]
        # lhand_pos = pb.getLinkState(robot, crab_link_idx.front_left__foot_link)[0]
        # rhand_pos = pb.getLinkState(robot, crab_link_idx.front_right__foot_link)[0]

        # # Compute the vector from the cylinder to each end effector
        # lfoot_cyl_vector = np.array(cylinder_pos) - np.array(lfoot_pos) 
        # rfoot_cyl_vector = np.array(cylinder_pos) - np.array(rfoot_pos) 
        # lhand_cyl_vector = np.array(cylinder_pos) - np.array(lhand_pos) 
        # rhand_cyl_vector = np.array(cylinder_pos) - np.array(rhand_pos) 
        
        # rpc_crab_sensor_data.lfoot_target_vector_ = lfoot_cyl_vector 
        # rpc_crab_sensor_data.rfoot_target_vector_ = rfoot_cyl_vector 
        # rpc_crab_sensor_data.lhand_target_vector_ = lhand_cyl_vector 
        # rpc_crab_sensor_data.rhand_target_vector_ = rhand_cyl_vector  
        
        # get position to target 
        base_pos = pb.getLinkState(robot, crab_link_idx.base_link)[0]
        
        body_target_vector = np.array(target_pos) - np.array(base_pos) 
        rpc_crab_sensor_data.body_target_vector_ = body_target_vector

        # ----------------------------------
        # compute control command
        # ----------------------------------

        if Config.MEASURE_COMPUTATION_TIME:
            timer.tic()

        rpc_crab_interface.GetCommand(rpc_crab_sensor_data, rpc_crab_command)

        if Config.MEASURE_COMPUTATION_TIME:
            comp_time = timer.tocvalue()
            compuation_cal_list.append(comp_time)

        # copy command data from rpc command class
        rpc_trq_command = rpc_crab_command.joint_trq_cmd_
        rpc_joint_pos_command = rpc_crab_command.joint_pos_cmd_
        rpc_joint_vel_command = rpc_crab_command.joint_vel_cmd_ 
        
        apply_control_input_to_pybullet(robot, rpc_trq_command) 
        
        # use rpc_joint_pos_command and vel_command in PD controller 
        # local impedance controller 
        
        # save current torso velocity for next iteration
        previous_torso_velocity = pybullet_util.get_link_vel(
            robot, crab_link_idx.base_link
        )[3:6]
        
        # # Apply an external force to the robot
        # force = [0, 0, 1]  # Example force vector in the x-direction
        # position = base_com_pos  # Apply force at the center of mass
        # link_index = -1  # Apply force to the base link
        # pb.applyExternalForce(robot, link_index, force, position, pb.WORLD_FRAME)
        
        # # get position of green ball 
        # target_pos, _ = pb.getBasePositionAndOrientation(green_ball) 
        
        # # Get the current position and orientation of the robot's base link
        # base_pos, base_ori = pb.getBasePositionAndOrientation(robot)
        # base_pos = np.array(base_pos)
        # base_ori = np.array(pb.getMatrixFromQuaternion(base_ori)).reshape(3, 3)
        
        # # Get the current direction of z_neg_arrow (negative z-axis of the base)
        # z_neg = -base_ori[:, 2]
        
        # # Compute the direction toward the target
        # direction_to_target = target_pos - base_pos
        # direction_to_target /= np.linalg.norm(direction_to_target)  # Normalize the vector

        # # Compute the error (angle between z_neg_arrow and direction_to_target)
        # error = np.arccos(np.clip(np.dot(z_neg, direction_to_target), -1.0, 1.0))

        # # Compute the torque using the PID controller
        # dt = 1.0 / 240.0  # Assuming the simulation step is 1/240 seconds
        # torque_magnitude = pid.compute(error, dt)

        # # Compute the torque direction (cross product to get the rotation axis)
        # torque_direction = np.cross(z_neg, direction_to_target)
        # torque_direction /= np.linalg.norm(torque_direction)  # Normalize the vector

        # # Apply the torque to the robot's base link
        # torque = torque_magnitude * torque_direction
        # pb.applyExternalTorque(robot, -1, torque, pb.WORLD_FRAME)

        # ----------------------------------
        # Save Image file
        # ----------------------------------

        if (Config.VIDEO_RECORD) and (count % Config.RECORD_FREQ == 0):
            camera_data = pb.getDebugVisualizerCamera()
            frame = pybullet_util.get_camera_image_from_debug_camera(
                camera_data, Config.RENDER_WIDTH, Config.RENDER_HEIGHT
            )
            filename = video_dir + "/step%06d.jpg" % jpg_count
            cv2.imwrite(filename, frame)
            jpg_count += 1

        pb.stepSimulation()  # step simulation

        count += 1
