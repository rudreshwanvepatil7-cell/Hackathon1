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
# sim functions
# ----------------------------------

if Config.MEASURE_COMPUTATION_TIME:
    from pytictoc import TicToc


def signal_handler(signal, frame):
    if Config.MEASURE_COMPUTATION_TIME:
        print("========================================================")
        print('saving list of compuation time in "compuation_time.txt"')
        print("========================================================")
        np.savetxt(
            "computation_time.txt", np.array([compuation_cal_list]), delimiter=","
        )

    if Config.VIDEO_RECORD:
        print("========================================================")
        print("Making Video")
        print("========================================================")
        pybullet_util.make_video(video_dir)

    pb.disconnect()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

# ----------------------------------
# main simulation loop
# ----------------------------------

if __name__ == "__main__":
    ## connect pybullet sim server
    pb.connect(pb.GUI)
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

    # base_pos = [0.0, -10.0, 10.0]
    base_pos = Config.INITIAL_BASE_JOINT_POS
    pb.resetDebugVisualizerCamera(
        cameraDistance=12.5,
        cameraYaw=120,
        cameraPitch=-45,
        cameraTargetPosition=base_pos + np.array([0.5, 0.3, -base_pos[2] + 1.5]),
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
    # initial_linear_velocity = [0.0, 0.0, 0.0]  # Example: zero initial velocity
    initial_linear_velocity = [0.0, 0.4, -0.2]  # Example: zero initial velocity
    initial_angular_velocity = [0.0, 0.0, 0.0]  # No initial angular velocity

    pb.resetBaseVelocity(
        robot,
        linearVelocity=initial_linear_velocity,
        angularVelocity=initial_angular_velocity,
    )

    cylinder_robot = pb.loadURDF(
        cwd + "/robot_model/cylinder.urdf",
        [0.0, 0.0, 2.5],
        [0, 0.707, 0, 0.707],
        useFixedBase=True,
    )
    
    # get cylinder position 
    cylinder_pos, _ = pb.getBasePositionAndOrientation(cylinder_robot)

    ground = pb.loadURDF(cwd + "/robot_model/ground/plane.urdf", useFixedBase=True)

    # Create a large plane to simulate a background
    # plane_shape = pb.createCollisionShape(pb.GEOM_PLANE)
    # plane_visual = pb.createVisualShape(pb.GEOM_PLANE, rgbaColor=[0, 0, 0, 1])
    # plane = pb.createMultiBody(0, plane_shape, plane_visual)

    # # Position the plane far away to act as a background
    # pb.resetBasePositionAndOrientation(plane, [0, 0, -100], [0, 0, 0, 1])

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
    set_init_config_pybullet_robot(robot)

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

    while True:
        # ----------------------------------
        # Moving Camera Setting
        # ----------------------------------

        base_pos, base_ori = pb.getBasePositionAndOrientation(robot)
        pb.resetDebugVisualizerCamera(cameraDistance=12.5,
                                      cameraYaw=120,
                                      cameraPitch=-45,
                                      cameraTargetPosition=base_pos +
                                      np.array([0.5, 0.3, -base_pos[2] + 1.5]))

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

        apply_magnetic_force_to_foot(
            robot,
            cylinder_robot,
            crab_link_idx.front_right__cluster_3_wrist_link,
        )
        apply_magnetic_force_to_foot(
            robot,
            cylinder_robot,
            crab_link_idx.front_left__cluster_3_wrist_link,
        )
        apply_magnetic_force_to_foot(
            robot,
            cylinder_robot,
            crab_link_idx.back_right__cluster_3_wrist_link,
        )
        apply_magnetic_force_to_foot(
            robot,
            cylinder_robot,
            crab_link_idx.back_left__cluster_3_wrist_link,
        )

        # ----------------------------------
        # Get Keyboard Event
        # ----------------------------------

        keys = pb.getKeyboardEvents()
        if pybullet_util.is_key_triggered(keys, "1"):
            pass

        # ----------------------------------
        # Get Sensor Data
        # ----------------------------------

        (
            imu_frame_quat,
            imu_ang_vel,
            imu_dvel,
            joint_pos,
            joint_vel,
            b_FL_foot_contact,
            b_FR_foot_contact,
            b_RL_foot_contact,
            b_RR_foot_contact,
            FL_normal_force,
            FR_normal_force,
            RL_normal_force,
            RR_normal_force,
        ) = get_sensor_data_from_pybullet(
            robot, previous_torso_velocity=previous_torso_velocity
        )

        ## copy sensor data to rpc sensor data class
        rpc_crab_sensor_data.imu_frame_quat_ = imu_frame_quat
        rpc_crab_sensor_data.imu_ang_vel_ = imu_ang_vel
        rpc_crab_sensor_data.imu_dvel_ = imu_dvel
        rpc_crab_sensor_data.imu_lin_acc_ = imu_dvel / dt
        rpc_crab_sensor_data.joint_pos_ = joint_pos
        rpc_crab_sensor_data.joint_vel_ = joint_vel
        rpc_crab_sensor_data.b_FL_foot_contact_ = b_FL_foot_contact
        rpc_crab_sensor_data.b_FR_foot_contact_ = b_FR_foot_contact
        rpc_crab_sensor_data.b_RL_foot_contact_ = b_RL_foot_contact
        rpc_crab_sensor_data.b_RR_foot_contact_ = b_RR_foot_contact
        rpc_crab_sensor_data.FL_normal_force_ = FL_normal_force
        rpc_crab_sensor_data.FR_normal_force_ = FR_normal_force
        rpc_crab_sensor_data.RL_normal_force_ = RL_normal_force
        rpc_crab_sensor_data.RR_normal_force_ = RR_normal_force

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

        # rpc_trq_command = np.ones(28) * 0.15
        # pdb.set_trace()

        # apply command to pybullet robot
        apply_control_input_to_pybullet(robot, rpc_trq_command)

        # pdb.set_trace()

        # save current torso velocity for next iteration
        previous_torso_velocity = pybullet_util.get_link_vel(
            robot, crab_link_idx.base_link
        )[3:6]

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
