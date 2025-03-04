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

def get_joint_data(robot, joint_id):

    joint_info  = pb.getJointInfo(robot, joint_id)
    joint_state = pb.getJointState(robot, joint_id) 
    return {
        'joint_id': joint_id,
        'joint_name': joint_info[1].decode('utf-8'),
        'position': joint_state[0],
        'velocity': joint_state[1], 
        # 'torque': joint_state[3],
        'lower_limit': joint_info[8],
        'upper_limit': joint_info[9]
    }

def get_joint_ids():
    # Get all joint indices from crab_joint_idx class
    joint_ids = [value for value in vars(crab_joint_idx).values() 
                if isinstance(value, int)]  # Only get the integer values
    joint_ids.sort()  # Sort them in ascending order
    return joint_ids 

def get_joints_data(robot, rpc_trq_command):

    joints_data = {}
    joint_ids = get_joint_ids()

    for joint_id in joint_ids:
        joint_data            = get_joint_data(robot, joint_id) 
        joint_data['torque']  = get_rpc_trq_from_joint_id(rpc_trq_command, joint_id)
        joints_data[joint_id] = joint_data 

    return joints_data 

def get_joint_limits(robot, joint_id):
    joint_limits = pb.getJointInfo(robot, joint_id)
    return joint_limits[8], joint_limits[9] 

def get_joint_hist(joints_hist_raw, joint_id):

    # Initialize dictionary to store joint_hist
    joint_hist = {
        'joint_id': joint_id,
        'sim_time': [],
        'pos': [], 
        'vel': [],
        'torque': []
    }

    # Populate joint_hist from each timestep
    for time_step in joints_hist_raw:

        joint_hist['sim_time'].append(time_step['sim_time'])

        joint_data = time_step['joints'][joint_id]
        joint_hist['pos'].append(joint_data['position'])
        joint_hist['vel'].append(joint_data['velocity']) 
        joint_hist['torque'].append(joint_data['torque'])

    return joint_hist

def get_all_joints_hist(joints_hist_raw):
    joints_hist = {}
    for joint_id in get_joint_ids():
        joints_hist[joint_id] = get_joint_hist(joints_hist_raw, joint_id)
    return joints_hist

# ---------------------------------- 
# reset joint angles 
# ---------------------------------- 

def set_0_config_robot(robot):
    """
    Reset all joints to zero configuration using correct PyBullet joint IDs
    """
    # Front Left Leg
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_1_roll, np.radians(0), 0.0)    # jointId = 26
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_1_pitch, np.radians(0), 0.0)   # jointId = 27
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_2_roll, np.radians(0), 0.0)    # jointId = 30
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_2_pitch, np.radians(0), 0.0)   # jointId = 31
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_3_roll, np.radians(0), 0.0)    # jointId = 34
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_3_pitch, np.radians(0), 0.0)   # jointId = 35
    pb.resetJointState(robot, crab_joint_idx.front_left__cluster_3_wrist, np.radians(0), 0.0)   # jointId = 37
    
    # Front Right Leg
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_1_roll, np.radians(0), 0.0)   # jointId = 10
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_1_pitch, np.radians(0), 0.0)  # jointId = 11
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_2_roll, np.radians(0), 0.0)   # jointId = 14
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_2_pitch, np.radians(0), 0.0)  # jointId = 15
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_3_roll, np.radians(0), 0.0)   # jointId = 18
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_3_pitch, np.radians(0), 0.0)  # jointId = 19
    pb.resetJointState(robot, crab_joint_idx.front_right__cluster_3_wrist, np.radians(0), 0.0)  # jointId = 21

    # Back Left Leg
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_1_roll, np.radians(0), 0.0)     # jointId = 58
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_1_pitch, np.radians(0), 0.0)    # jointId = 59
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_2_roll, np.radians(0), 0.0)     # jointId = 62
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_2_pitch, np.radians(0), 0.0)    # jointId = 63
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_3_roll, np.radians(0), 0.0)     # jointId = 66
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_3_pitch, np.radians(0), 0.0)    # jointId = 67
    pb.resetJointState(robot, crab_joint_idx.back_left__cluster_3_wrist, np.radians(0), 0.0)    # jointId = 69

    # Back Right Leg
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_1_roll, np.radians(0), 0.0)    # jointId = 42
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_1_pitch, np.radians(0), 0.0)   # jointId = 43
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_2_roll, np.radians(0), 0.0)    # jointId = 46
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_2_pitch, np.radians(0), 0.0)   # jointId = 47
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_3_roll, np.radians(0), 0.0)    # jointId = 50
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_3_pitch, np.radians(0), 0.0)   # jointId = 51
    pb.resetJointState(robot, crab_joint_idx.back_right__cluster_3_wrist, np.radians(0), 0.0)   # jointId = 53


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

def get_rpc_trq_from_joint_id(rpc_trq_command, joint_id):
    """
    Get the appropriate torque command from rpc_trq_command array for a given PyBullet joint_id
    
    Args:
        rpc_trq_command: Array of 28 torque commands
        joint_id: PyBullet joint ID to get torque for
        
    Returns:
        The torque command for the specified joint
    """
    # Map from PyBullet joint_id to rpc_trq_command index
    joint_id_to_cmd_idx = {
        # Front Left Leg
        crab_joint_idx.front_left__cluster_1_roll: 0,     # jointId 26 -> index 0
        crab_joint_idx.front_left__cluster_1_pitch: 1,    # jointId 27 -> index 1
        crab_joint_idx.front_left__cluster_2_roll: 2,     # jointId 30 -> index 2
        crab_joint_idx.front_left__cluster_2_pitch: 3,    # jointId 31 -> index 3
        crab_joint_idx.front_left__cluster_3_roll: 4,     # jointId 34 -> index 4
        crab_joint_idx.front_left__cluster_3_pitch: 5,    # jointId 35 -> index 5
        crab_joint_idx.front_left__cluster_3_wrist: 6,    # jointId 37 -> index 6
        
        # Front Right Leg
        crab_joint_idx.front_right__cluster_1_roll: 7,    # jointId 10 -> index 7
        crab_joint_idx.front_right__cluster_1_pitch: 8,   # jointId 11 -> index 8
        crab_joint_idx.front_right__cluster_2_roll: 9,    # jointId 14 -> index 9
        crab_joint_idx.front_right__cluster_2_pitch: 10,  # jointId 15 -> index 10
        crab_joint_idx.front_right__cluster_3_roll: 11,   # jointId 18 -> index 11
        crab_joint_idx.front_right__cluster_3_pitch: 12,  # jointId 19 -> index 12
        crab_joint_idx.front_right__cluster_3_wrist: 13,  # jointId 21 -> index 13
        
        # Back Left Leg
        crab_joint_idx.back_left__cluster_1_roll: 14,     # jointId 58 -> index 14
        crab_joint_idx.back_left__cluster_1_pitch: 15,    # jointId 59 -> index 15
        crab_joint_idx.back_left__cluster_2_roll: 16,     # jointId 62 -> index 16
        crab_joint_idx.back_left__cluster_2_pitch: 17,    # jointId 63 -> index 17
        crab_joint_idx.back_left__cluster_3_roll: 18,     # jointId 66 -> index 18
        crab_joint_idx.back_left__cluster_3_pitch: 19,    # jointId 67 -> index 19
        crab_joint_idx.back_left__cluster_3_wrist: 20,    # jointId 69 -> index 20
        
        # Back Right Leg
        crab_joint_idx.back_right__cluster_1_roll: 21,    # jointId 42 -> index 21
        crab_joint_idx.back_right__cluster_1_pitch: 22,   # jointId 43 -> index 22
        crab_joint_idx.back_right__cluster_2_roll: 23,    # jointId 46 -> index 23
        crab_joint_idx.back_right__cluster_2_pitch: 24,   # jointId 47 -> index 24
        crab_joint_idx.back_right__cluster_3_roll: 25,    # jointId 50 -> index 25
        crab_joint_idx.back_right__cluster_3_pitch: 26,   # jointId 51 -> index 26
        crab_joint_idx.back_right__cluster_3_wrist: 27,   # jointId 53 -> index 27
    }
    
    # Get the index for this joint_id
    cmd_idx = joint_id_to_cmd_idx.get(joint_id)
    
    # Return the corresponding torque command
    return rpc_trq_command[cmd_idx]

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

def update_arrows(base_com_pos, rot_world_basecom, target_pos, x_arrow=None, y_arrow=None, z_arrow=None, z_neg_arrow=None, target_arrow=None):
    
    arrow_start_pos = base_com_pos
    x_end = base_com_pos + 2 * rot_world_basecom[:, 0]
    y_end = base_com_pos + 2 * rot_world_basecom[:, 1]
    z_end = base_com_pos + 2 * rot_world_basecom[:, 2]
    z_neg = base_com_pos - 2 * rot_world_basecom[:, 2]
    
    target_arrow = create_arrow(arrow_start_pos, target_pos, color=[1, 1, 0], replace_id=target_arrow) 

    x_arrow = create_arrow(arrow_start_pos, x_end, replace_id=x_arrow)
    y_arrow = create_arrow(arrow_start_pos, y_end, color=[0, 1, 0], replace_id=y_arrow)
    z_arrow = create_arrow(arrow_start_pos, z_end, color=[0, 0, 1], replace_id=z_arrow)
    z_neg_arrow = create_arrow(arrow_start_pos, z_neg, color=[0, 1, 1], replace_id=z_neg_arrow)

    return x_arrow, y_arrow, z_arrow, z_neg_arrow, target_arrow 

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
