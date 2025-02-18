import pybullet as pb
import os
import sys
import numpy as np 
import signal 

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  # include pybind module

from config.crab.sim.pybullet.ihwbc.pybullet_params import *
from util.python_utils import pybullet_util 

# ---------------------------------- 
# debug joint state 
# ---------------------------------- 

def get_joint_ori(robot, link_id): 

    # joint_id = crab_link_idx.front_left__foot_link
    
    # Get orientation of a specific link (e.g. front left foot)
    # getLinkState returns position and orientation in world frame
    link_state = pb.getLinkState(robot, link_id)
    link_pos   = link_state[0]  # Position in world frame
    link_quat  = link_state[1]  # Orientation as quaternion in world frame
    link_dcm   = np.array(pb.getMatrixFromQuaternion(link_quat)).reshape(3,3)  # Convert to rotation matrix
    
    return link_pos, link_quat 
    
def debug_joint_properties(robot, joint_id):
    joint_info = pb.getJointInfo(robot, joint_id)
    print(f"Joint {joint_info[1]}: ")
    print(f"  Type: {joint_info[2]}")
    print(f"  Damping: {joint_info[6]}")
    print(f"  Friction: {joint_info[7]}")
    print(f"  Lower limit: {joint_info[8]}")
    print(f"  Upper limit: {joint_info[9]}")
    print(f"  Max force: {joint_info[10]}")
    print(f"  Max velocity: {joint_info[11]}") 

# example usage: 
#   joint_id = crab_joint_idx.front_left__cluster_1_pitch
#   debug_joint_properties(robot, joint_id)

def print_joint_state(robot, joint_id):
    joint_state = pb.getJointState(robot, joint_id)
    joint_info = pb.getJointInfo(robot, joint_id)
    joint_name = joint_info[1].decode('utf-8')
    
    print(f"\nJoint {joint_name} (id: {joint_id}) state:")
    print(f"Position: {joint_state[0]:.3f} rad ({np.degrees(joint_state[0]):.1f} deg)")
    print(f"Velocity: {joint_state[1]:.3f} rad/s")
    print(f"Reaction forces: {joint_state[2]}")
    print(f"Applied motor torque: {joint_state[3]:.3f} N⋅m")

def get_joint(robot, joint_id):

    joint_info  = pb.getJointInfo(robot, joint_id)
    joint_state = pb.getJointState(robot, joint_id) 
    return {
        'joint_id': joint_id,
        'joint_name': joint_info[1].decode('utf-8'),
        'position': joint_state[0],
        'velocity': joint_state[1], 
        'torque': joint_state[3],
        'lower_limit': joint_info[8],
        'upper_limit': joint_info[9]
    }

def get_joints(robot):

    joint_pos_data = []
    # Get all joint indices from crab_joint_idx class
    joint_ids = [value for value in vars(crab_joint_idx).values() 
                if isinstance(value, int)]  # Only get the integer values
    joint_ids.sort()  # Sort them in ascending order
    
    for joint_id in joint_ids:
        joint_pos_data.append(get_joint(robot, joint_id))
    return joint_pos_data

def get_joint_pos_hist(joints_hist, joint_id):

    # for all time steps in joints_hist, get the sim time and joints
    sim_time_hist = []
    joint_pos_hist = []
    for time_step in joints_hist:
        sim_time = time_step['sim_time'] 
        joints   = time_step['joints']
        joint_pos = joints[joint_id]['position']
        sim_time_hist.append(sim_time)
        joint_pos_hist.append(joint_pos)
    return sim_time_hist, joint_pos_hist

# ---------------------------------- 
# reset joint angles 
# ---------------------------------- 

def set_0_config_robot(robot):
    
    # Front Left Limb (4 DOF)
    pb.resetJointState(robot, 0, np.radians(0), 0.0)  # cluster 1 roll 
    pb.resetJointState(robot, 1, np.radians(0), 0.0)  # cluster 1 pitch 
    pb.resetJointState(robot, 2, np.radians(0), 0.0)  # cluster 2 roll 
    pb.resetJointState(robot, 3, np.radians(0), 0.0)  # cluster 2 pitch 
    pb.resetJointState(robot, 4, np.radians(0), 0.0)  # cluster 3 roll 
    pb.resetJointState(robot, 5, np.radians(0), 0.0)  # cluster 3 pitch 
    pb.resetJointState(robot, 6, np.radians(0), 0.0)  # cluster 3 wrist 
    
    # Front Right 
    pb.resetJointState(robot, 7, np.radians(0), 0.0)  # cluster 1 roll 
    pb.resetJointState(robot, 8, np.radians(0), 0.0)  # cluster 1 pitch 
    pb.resetJointState(robot, 9, np.radians(0), 0.0)  # cluster 2 roll 
    pb.resetJointState(robot, 10, np.radians(0), 0.0) # cluster 2 pitch 
    pb.resetJointState(robot, 11, np.radians(0), 0.0) # cluster 3 roll 
    pb.resetJointState(robot, 12, np.radians(0), 0.0) # cluster 3 pitch 
    pb.resetJointState(robot, 13, np.radians(0), 0.0) # cluster 3 wrist 

    # Back Left 
    pb.resetJointState(robot, 14, np.radians(0), 0.0) # cluster 1 roll 
    pb.resetJointState(robot, 15, np.radians(0), 0.0) # cluster 1 pitch 
    pb.resetJointState(robot, 16, np.radians(0), 0.0) # cluster 2 roll 
    pb.resetJointState(robot, 17, np.radians(0), 0.0) # cluster 2 pitch 
    pb.resetJointState(robot, 18, np.radians(0), 0.0) # cluster 3 roll 
    pb.resetJointState(robot, 19, np.radians(0), 0.0) # cluster 3 pitch 
    pb.resetJointState(robot, 20, np.radians(0), 0.0) # cluster 3 wrist 

    # Back Right 
    pb.resetJointState(robot, 21, np.radians(0), 0.0) # cluster 1 roll 
    pb.resetJointState(robot, 22, np.radians(0), 0.0) # cluster 1 pitch 
    pb.resetJointState(robot, 23, np.radians(0), 0.0) # cluster 2 roll 
    pb.resetJointState(robot, 24, np.radians(0), 0.0) # cluster 2 pitch 
    pb.resetJointState(robot, 25, np.radians(0), 0.0) # cluster 3 roll 
    pb.resetJointState(robot, 26, np.radians(0), 0.0) # cluster 3 pitch 
    pb.resetJointState(robot, 27, np.radians(0), 0.0) # cluster 3 wrist 


# ---------------------------------- 
# test range of motion 
# ---------------------------------- 

def generate_momentum_test_sequence():
    # Sequence of joint commands to create maximum angular momentum
    sequences = []
    
    # Phase 1: Extend all limbs outward (like a starfish)
    extend_limbs = np.zeros(28)  # 28 joint DOFs
    extend_limbs[[2,3,4,5, 16,17,18,19]] = 0.5  # Leg joints
    extend_limbs[[8,9,10,11, 22,23,24,25]] = 0.5  # Arm joints
    sequences.append((extend_limbs, 1.0))  # (command, duration)
    
    # Phase 2: Rotate limbs in same direction to build angular momentum
    twist_limbs = np.zeros(28)
    twist_limbs[[1,7,15,21]] = 1.0  # First joint of each limb
    sequences.append((twist_limbs, 0.5))
    
    # Phase 3: Quickly pull limbs in to increase rotation speed
    contract_limbs = np.zeros(28)
    contract_limbs[[2,3,4,5, 16,17,18,19]] = -0.5  # Leg joints
    contract_limbs[[8,9,10,11, 22,23,24,25]] = -0.5  # Arm joints
    sequences.append((contract_limbs, 0.3))
    
    return sequences

def run_momentum_test(robot, dt):
    # robot = init_robot()  # Initialize robot
    sequences = generate_momentum_test_sequence()
    
    for command, duration in sequences:
        steps = int(duration * 1/dt)  # Assuming 240Hz simulation
        for _ in range(steps):
            apply_control_input_to_pybullet(robot, command)
            pb.stepSimulation()
            
            # Record orientation for analysis
            pos, ori = pb.getBasePositionAndOrientation(robot)
            rot = np.array(pb.getMatrixFromQuaternion(ori)).reshape(3,3)
            z_axis = rot[:,2]
            print(f"Current Z-axis orientation: {z_axis}")

# ---------------------------------- 
# PID controller 
# ---------------------------------- 

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# ---------------------------------- 
# functions to interact with pybullet 
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

    imu_dvel = pybullet_util.simulate_dVel_data(robot, crab_link_idx.base_link, previous_torso_velocity)

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
        bodyA=robot, linkIndexA=crab_link_idx.front_left__foot_link
    )
    for contact in contacts:
        # add z-component on all points of contact
        FL_normal_force += contact[9]

    FR_normal_force = 0
    contacts = pb.getContactPoints(
        bodyA=robot, linkIndexA=crab_link_idx.front_right__foot_link
    )
    for contact in contacts:
        # add z-component on all points of contact
        FR_normal_force += contact[9]

    RL_normal_force = 0
    contacts = pb.getContactPoints(
        bodyA=robot, linkIndexA=crab_link_idx.back_left__foot_link
    )
    for contact in contacts:
        # add z-component on all points of contact
        RL_normal_force += contact[9]

    RR_normal_force = 0
    contacts = pb.getContactPoints(
        bodyA=robot, linkIndexA=crab_link_idx.back_right__foot_link
    )
    for contact in contacts:
        # add z-component on all points of contact
        RR_normal_force += contact[9]

    # Determine foot contact states based on the z-coordinate of the foot link
    b_FL_foot_contact = FL_normal_force >= 0.05
    b_FR_foot_contact = FR_normal_force >= 0.05
    b_RL_foot_contact = RL_normal_force >= 0.05
    b_RR_foot_contact = RR_normal_force >= 0.05

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

def get_assign_sensor_data(robot, rpc_crab_sensor_data, dt, previous_torso_velocity):
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
    
    return rpc_crab_sensor_data 

# ----------------------------------
    
def create_arrow(start_pos, end_pos, color=[1, 0, 0], line_width=2, replace_id=None): 
    
    if replace_id == None: 
        return pb.addUserDebugLine(start_pos, end_pos, lineColorRGB=color, lineWidth=line_width)
    else: 
        return pb.addUserDebugLine(start_pos, end_pos, lineColorRGB=color, lineWidth=line_width, replaceItemUniqueId=replace_id)

def update_arrows(base_com_pos, rot_world_basecom, x_arrow=None, y_arrow=None, z_arrow=None, z_neg_arrow=None):
    arrow_start_pos = base_com_pos
    x_end = base_com_pos + 2 * rot_world_basecom[:, 0]
    y_end = base_com_pos + 2 * rot_world_basecom[:, 1]
    z_end = base_com_pos + 2 * rot_world_basecom[:, 2]
    z_neg = base_com_pos - 2 * rot_world_basecom[:, 2]

    x_arrow = create_arrow(arrow_start_pos, x_end, replace_id=x_arrow)
    y_arrow = create_arrow(arrow_start_pos, y_end, color=[0, 1, 0], replace_id=y_arrow)
    z_arrow = create_arrow(arrow_start_pos, z_end, color=[0, 0, 1], replace_id=z_arrow)
    z_neg_arrow = create_arrow(arrow_start_pos, z_neg, color=[0, 1, 1], replace_id=z_neg_arrow)

    return x_arrow, y_arrow, z_arrow, z_neg_arrow

# ---------------------------------- 

def apply_control_input_to_pybullet(robot, command):
    mode = pb.TORQUE_CONTROL

    # FRONT LEFT
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_left__cluster_1_roll,
        controlMode=mode,
        force=command[0],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_left__cluster_1_pitch,
        controlMode=mode,
        force=command[1],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_left__cluster_2_roll,
        controlMode=mode,
        force=command[2],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_left__cluster_2_pitch,
        controlMode=mode,
        force=command[3],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_left__cluster_3_roll,
        controlMode=mode,
        force=command[4],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_left__cluster_3_pitch,
        controlMode=mode,
        force=command[5],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_left__cluster_3_wrist,
        controlMode=mode,
        force=command[6],
    )

    # FRONT RIGHT
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_right__cluster_1_roll,
        controlMode=mode,
        force=command[7],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_right__cluster_1_pitch,
        controlMode=mode,
        force=command[8],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_right__cluster_2_roll,
        controlMode=mode,
        force=command[9],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_right__cluster_2_pitch,
        controlMode=mode,
        force=command[10],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_right__cluster_3_roll,
        controlMode=mode,
        force=command[11],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_right__cluster_3_pitch,
        controlMode=mode,
        force=command[12],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.front_right__cluster_3_wrist,
        controlMode=mode,
        force=command[13],
    )

    # BACK LEFT
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_left__cluster_1_roll,
        controlMode=mode,
        force=command[14],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_left__cluster_1_pitch,
        controlMode=mode,
        force=command[15],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_left__cluster_2_roll,
        controlMode=mode,
        force=command[16],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_left__cluster_2_pitch,
        controlMode=mode,
        force=command[17],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_left__cluster_3_roll,
        controlMode=mode,
        force=command[18],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_left__cluster_3_pitch,
        controlMode=mode,
        force=command[19],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_left__cluster_3_wrist,
        controlMode=mode,
        force=command[20],
    )

    # BACK RIGHT
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_right__cluster_1_roll,
        controlMode=mode,
        force=command[21],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_right__cluster_1_pitch,
        controlMode=mode,
        force=command[22],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_right__cluster_2_roll,
        controlMode=mode,
        force=command[23],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_right__cluster_2_pitch,
        controlMode=mode,
        force=command[24],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_right__cluster_3_roll,
        controlMode=mode,
        force=command[25],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_right__cluster_3_pitch,
        controlMode=mode,
        force=command[26],
    )
    pb.setJointMotorControl2(
        robot,
        crab_joint_idx.back_right__cluster_3_wrist,
        controlMode=mode,
        force=command[27],
    )


# ----------------------------------


def set_init_config_pybullet_robot(robot):
    roll_angle = 0
    pitch_angle = -45

    # FRONT LEFT
    pb.resetJointState(
        robot, crab_joint_idx.front_left__cluster_1_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_left__cluster_1_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_left__cluster_2_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_left__cluster_2_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_left__cluster_3_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_left__cluster_3_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_left__cluster_3_wrist, np.radians(-roll_angle), 0.0
    )

    # FRONT RIGHT
    pb.resetJointState(
        robot, crab_joint_idx.front_right__cluster_1_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_right__cluster_1_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_right__cluster_2_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_right__cluster_2_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_right__cluster_3_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_right__cluster_3_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.front_right__cluster_3_wrist, np.radians(-roll_angle), 0.0
    )

    # REAR LEFT
    pb.resetJointState(
        robot, crab_joint_idx.back_left__cluster_1_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_left__cluster_1_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_left__cluster_2_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_left__cluster_2_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_left__cluster_3_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_left__cluster_3_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_left__cluster_3_wrist, np.radians(-roll_angle), 0.0
    )

    # REAR RIGHT
    pb.resetJointState(
        robot, crab_joint_idx.back_right__cluster_1_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_right__cluster_1_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_right__cluster_2_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_right__cluster_2_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_right__cluster_3_roll, np.radians(roll_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_right__cluster_3_pitch, np.radians(pitch_angle), 0.0
    )
    pb.resetJointState(
        robot, crab_joint_idx.back_right__cluster_3_wrist, np.radians(-roll_angle), 0.0
    )


def apply_magnetic_force_to_foot(robot, satellite, link_idx):
    closest_points = pb.getClosestPoints(
        bodyA=robot, bodyB=satellite, distance=0.5, linkIndexA=link_idx, linkIndexB=-1
    )
    if not len(closest_points):
        return
    foot_closest = closest_points[0][5]
    sat_closest = closest_points[0][6]

    force = np.array(sat_closest) - np.array(foot_closest)
    force = 100 * force / np.linalg.norm(force)

    pb.applyExternalForce(
        objectUniqueId=robot,
        linkIndex=link_idx,
        forceObj=force,
        posObj=foot_closest,
        flags=pb.WORLD_FRAME,
    )


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
