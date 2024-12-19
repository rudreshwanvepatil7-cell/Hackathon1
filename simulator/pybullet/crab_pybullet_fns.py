import pybullet as pb
import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  # include pybind module

from config.crab.sim.pybullet.wbic.pybullet_params import *
from util.python_utils import pybullet_util
from util.python_utils import util
from util.python_utils import liegroup

# import signal
# import shutil
# import cv2

# # import go2_interface_py 
# import crab_interface_py  

# import pdb 

# ---------------------------------- 

def get_sensor_data_from_pybullet(robot, previous_torso_velocity): 
    
    # follow pinocchio robotsystem urdf reading convention
    joint_pos, joint_vel = np.zeros(28), np.zeros(28)

    imu_frame_quat = np.array(pb.getLinkState(robot, crab_link_idx.base_link, 1, 1)[1]) 

    # Front left  
    joint_pos[0] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_1_roll)[0]
    joint_pos[1] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_1_pitch)[0]
    joint_pos[2] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_2_roll)[0]
    joint_pos[3] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_2_pitch)[0] 
    joint_pos[4] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_3_roll)[0]
    joint_pos[5] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_3_pitch)[0]
    joint_pos[6] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_3_wrist)[0] 
    
    # Front right  
    joint_pos[7] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_1_roll)[0]
    joint_pos[8] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_1_pitch)[0]
    joint_pos[9] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_2_roll)[0]
    joint_pos[10] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_2_pitch)[0] 
    joint_pos[11] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_3_roll)[0]
    joint_pos[12] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_3_pitch)[0]
    joint_pos[13] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_3_wrist)[0] 
    
    # Back left 
    joint_pos[14] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_1_roll)[0]
    joint_pos[15] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_1_pitch)[0]
    joint_pos[16] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_2_roll)[0]
    joint_pos[17] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_2_pitch)[0] 
    joint_pos[18] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_3_roll)[0]
    joint_pos[19] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_3_pitch)[0]
    joint_pos[20] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_3_wrist)[0] 
    
    # Back right  
    joint_pos[21] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_1_roll)[0]
    joint_pos[22] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_1_pitch)[0]
    joint_pos[23] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_2_roll)[0]
    joint_pos[24] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_2_pitch)[0] 
    joint_pos[25] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_3_roll)[0]
    joint_pos[26] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_3_pitch)[0]
    joint_pos[27] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_3_wrist)[0] 

    imu_ang_vel = np.array(pb.getLinkState(robot, crab_link_idx.base_link, 1, 1)[7])

    imu_dvel = pybullet_util.simulate_dVel_data(
        robot, crab_link_idx.base_link, previous_torso_velocity
    )
    
    # Front left  
    joint_vel[0] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_1_roll)[1]
    joint_vel[1] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_1_pitch)[1]
    joint_vel[2] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_2_roll)[1]
    joint_vel[3] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_2_pitch)[1] 
    joint_vel[4] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_3_roll)[1]
    joint_vel[5] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_3_pitch)[1]
    joint_vel[6] = pb.getJointState(robot, crab_joint_idx.front_left__cluster_3_wrist)[1] 
    
    # Front right  
    joint_vel[7] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_1_roll)[1]
    joint_vel[8] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_1_pitch)[1]
    joint_vel[9] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_2_roll)[1]
    joint_vel[10] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_2_pitch)[1] 
    joint_vel[11] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_3_roll)[1]
    joint_vel[12] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_3_pitch)[1]
    joint_vel[13] = pb.getJointState(robot, crab_joint_idx.front_right__cluster_3_wrist)[1] 
    
    # Back left 
    joint_vel[14] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_1_roll)[1]
    joint_vel[15] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_1_pitch)[1]
    joint_vel[16] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_2_roll)[1]
    joint_vel[17] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_2_pitch)[1] 
    joint_vel[18] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_3_roll)[1]
    joint_vel[19] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_3_pitch)[1]
    joint_vel[20] = pb.getJointState(robot, crab_joint_idx.back_left__cluster_3_wrist)[1] 
    
    # Back right  
    joint_vel[21] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_1_roll)[1]
    joint_vel[22] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_1_pitch)[1]
    joint_vel[23] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_2_roll)[1]
    joint_vel[24] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_2_pitch)[1] 
    joint_vel[25] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_3_roll)[1]
    joint_vel[26] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_3_pitch)[1]
    joint_vel[27] = pb.getJointState(robot, crab_joint_idx.back_right__cluster_3_wrist)[1] 

    # normal force measured on each foot
    FL_normal_force = 0
    contacts = pb.getContactPoints(
        bodyA = robot, 
        linkIndexA = crab_link_idx.front_left__foot_link
        )
    for contact in contacts:
        # add z-component on all points of contact
        FL_normal_force += contact[9]

    FR_normal_force = 0
    contacts = pb.getContactPoints(
        bodyA = robot, 
        linkIndexA = crab_link_idx.front_right__foot_link
        )
    for contact in contacts:
        # add z-component on all points of contact
        FR_normal_force += contact[9]

    RL_normal_force = 0
    contacts = pb.getContactPoints(
        bodyA = robot, 
        linkIndexA = crab_link_idx.back_left__foot_link
        )
    for contact in contacts:
        # add z-component on all points of contact
        RL_normal_force += contact[9]

    RR_normal_force = 0
    contacts = pb.getContactPoints(
        bodyA = robot, 
        linkIndexA = crab_link_idx.back_right__foot_link
        )
    for contact in contacts:
        # add z-component on all points of contact
        RR_normal_force += contact[9]

    # Determine foot contact states based on the z-coordinate of the foot link
    b_FL_foot_contact = (
        True
        if pb.getLinkState(robot, crab_link_idx.front_left__foot_link, 1, 1)[0][2] <= 0.05
        else False
    )
    b_FR_foot_contact = (
        True
        if pb.getLinkState(robot, crab_link_idx.front_right__foot_link, 1, 1)[0][2] <= 0.05
        else False
    )
    b_RL_foot_contact = (
        True
        if pb.getLinkState(robot, crab_link_idx.back_left__foot_link, 1, 1)[0][2] <= 0.05
        else False
    )
    b_RR_foot_contact = (
        True
        if pb.getLinkState(robot, crab_link_idx.back_right__foot_link, 1, 1)[0][2] <= 0.05
        else False
    )

    return (
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
    )

# ---------------------------------- 

def apply_control_input_to_pybullet(robot, command):
    mode = pb.TORQUE_CONTROL
    
    # FRONT LEFT
    pb.setJointMotorControl2(robot, crab_joint_idx.front_left__cluster_1_roll, controlMode=mode, force=command[0])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_left__cluster_1_pitch, controlMode=mode, force=command[1])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_left__cluster_2_roll, controlMode=mode, force=command[2])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_left__cluster_2_pitch, controlMode=mode, force=command[3])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_left__cluster_3_roll, controlMode=mode, force=command[4])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_left__cluster_3_pitch, controlMode=mode, force=command[5])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_left__cluster_3_wrist, controlMode=mode, force=command[6])
    
    # FRONT RIGHT
    pb.setJointMotorControl2(robot, crab_joint_idx.front_right__cluster_1_roll, controlMode=mode, force=command[7])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_right__cluster_1_pitch, controlMode=mode, force=command[8])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_right__cluster_2_roll, controlMode=mode, force=command[9])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_right__cluster_2_pitch, controlMode=mode, force=command[10])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_right__cluster_3_roll, controlMode=mode, force=command[11])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_right__cluster_3_pitch, controlMode=mode, force=command[12])
    pb.setJointMotorControl2(robot, crab_joint_idx.front_right__cluster_3_wrist, controlMode=mode, force=command[13])
    
    # BACK LEFT
    pb.setJointMotorControl2(robot, crab_joint_idx.back_left__cluster_1_roll, controlMode=mode, force=command[14])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_left__cluster_1_pitch, controlMode=mode, force=command[15])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_left__cluster_2_roll, controlMode=mode, force=command[16])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_left__cluster_2_pitch, controlMode=mode, force=command[17])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_left__cluster_3_roll, controlMode=mode, force=command[18])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_left__cluster_3_pitch, controlMode=mode, force=command[19])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_left__cluster_3_wrist, controlMode=mode, force=command[20])
    
    # BACK RIGHT
    pb.setJointMotorControl2(robot, crab_joint_idx.back_right__cluster_1_roll, controlMode=mode, force=command[21])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_right__cluster_1_pitch, controlMode=mode, force=command[22])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_right__cluster_2_roll, controlMode=mode, force=command[23])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_right__cluster_2_pitch, controlMode=mode, force=command[24])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_right__cluster_3_roll, controlMode=mode, force=command[25])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_right__cluster_3_pitch, controlMode=mode, force=command[26])
    pb.setJointMotorControl2(robot, crab_joint_idx.back_right__cluster_3_wrist, controlMode=mode, force=command[27])

# ---------------------------------- 

def set_init_config_pybullet_robot(robot):
    
    roll_angle  = 0 
    pitch_angle = -45 
    
    # FRONT LEFT 
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_1_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_1_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_2_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_2_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_3_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_3_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_3_wrist, np.radians(-roll_angle), 0.0) 
    
    # FRONT RIGHT 
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_1_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_1_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_2_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_2_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_3_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_3_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_3_wrist, np.radians(-roll_angle), 0.0) 
    
    # REAR LEFT 
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_1_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_1_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_2_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_2_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_3_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_3_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_3_wrist, np.radians(-roll_angle), 0.0) 
    
    # REAR RIGHT 
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_1_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_1_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_2_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_2_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_3_roll, np.radians(roll_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_3_pitch, np.radians(pitch_angle), 0.0) 
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_3_wrist, np.radians(-roll_angle), 0.0) 