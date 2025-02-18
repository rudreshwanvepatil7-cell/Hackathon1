import numpy as np
import matplotlib.pyplot as plt
import pickle
import argparse
from datetime import datetime

import pybullet as pb
import os
import sys
import numpy as np 
import signal 

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + "/build/lib")  # include pybind module

from config.crab.sim.pybullet.ihwbc.pybullet_params import * 
from simulator.pybullet.crab_pybullet_fns import * 
from util.python_utils import pybullet_util 

# ---------------------------------- 
# ---------------------------------- 

def plot_joint_pos_hist(robot, joint_hist, joint_id ):

    joint_pos = joint_hist['pos']
    joint_vel = joint_hist['vel'] 
    joint_torque = joint_hist['torque']
    sim_time = joint_hist['sim_time']

    # Get joint name from joint info
    joint_data = get_joint_data(robot, joint_id) 
    joint_name = joint_data['joint_name']

    joint_limits = get_joint_limits(robot, joint_id)

    print(f"joint_name = {joint_name}")
    print(f"joint_pos = {joint_pos}")
    print(f"joint_limits = {joint_limits}")

    plt.plot(sim_time, joint_pos)
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Position (rad)')
    
    # Add horizontal lines for joint limits
    lower_limit, upper_limit = joint_limits
    plt.axhline(y=upper_limit, color='r', linestyle='--', label='Upper Limit')
    plt.axhline(y=lower_limit, color='r', linestyle='--', label='Lower Limit')
    plt.legend()

    plt.title(f"{joint_name} Position vs Time")
    plt.show()

    return 

def subplot_joint_pos_hist(robot, joint_hist, joint_id, ax):

    # get the row of the axis 
    row = ax.get_subplotspec().rowspan.start
    if row != 6: 
        ax.set_xticklabels([])  # Remove x-axis tick labels

    joint_pos = joint_hist['pos']
    joint_vel = joint_hist['vel'] 
    joint_torque = joint_hist['torque']
    sim_time = joint_hist['sim_time']

    # Get joint name from joint info
    joint_data = get_joint_data(robot, joint_id) 
    joint_name = joint_data['joint_name']

    joint_limits = get_joint_limits(robot, joint_id)

    ax.plot(sim_time, joint_pos)
    
    # Add horizontal lines for joint limits
    lower_limit, upper_limit = joint_limits
    ax.axhline(y=upper_limit, color='r', linestyle='--', label='Upper Limit')
    ax.axhline(y=lower_limit, color='r', linestyle='--', label='Lower Limit')
    # ax.legend()

    ax.text(0.1, 0.90, joint_name, transform=ax.transAxes, 
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'),
            verticalalignment='top', fontsize=8)

    return ax 

def subplot_joint_vel_hist(robot, joint_hist, joint_id, ax):

    # get the row of the axis 
    row = ax.get_subplotspec().rowspan.start
    if row != 6: 
        ax.set_xticklabels([])  # Remove x-axis tick labels

    joint_pos = joint_hist['pos']
    joint_vel = joint_hist['vel'] 
    joint_torque = joint_hist['torque']
    sim_time = joint_hist['sim_time']

    # Get joint name from joint info
    joint_data = get_joint_data(robot, joint_id) 
    joint_name = joint_data['joint_name']

    joint_limits = get_joint_limits(robot, joint_id)

    ax.plot(sim_time, joint_vel)

    ax.text(0.1, 0.90, joint_name, transform=ax.transAxes, 
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'),
            verticalalignment='top', fontsize=8)

    return ax 

def subplot_joint_trq_hist(robot, joint_hist, joint_id, ax):

    # get the row of the axis 
    row = ax.get_subplotspec().rowspan.start
    if row != 6: 
        ax.set_xticklabels([])  # Remove x-axis tick labels

    joint_torque = joint_hist['torque']
    sim_time = joint_hist['sim_time']

    # Get joint name from joint info
    joint_data = get_joint_data(robot, joint_id) 
    joint_name = joint_data['joint_name']

    joint_limits = get_joint_limits(robot, joint_id)

    ax.plot(sim_time, joint_torque)
    
    ax.text(0.1, 0.90, joint_name, transform=ax.transAxes, 
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'),
            verticalalignment='top', fontsize=8)

    return ax 

def plot_limb_joints(robot, joints_hist_raw, set = 1):
    """Plot joint positions for a set of 7 joints from a limb.
    
    Args:
        robot: PyBullet robot instance
        joints_hist_raw: Raw joint history data
        set: Which set of 7 joints to plot (1-4)
            1 = Front left limb
            2 = Front right limb 
            3 = Back left limb
            4 = Back right limb
    """
    # Create figure with 7 subplots arranged vertically
    fig, axs = plt.subplots(7, 1, figsize=(10, 20))

    joint_ids = get_joint_ids()
    
    # Get slice based on which set of joints to plot
    if set == 1:
        joint_slice = slice(0, 7)
    elif set == 2:
        joint_slice = slice(7, 14) 
    elif set == 3:
        joint_slice = slice(14, 21)
    elif set == 4:
        joint_slice = slice(21, 28)
        # fig.suptitle('Back Right Limb Joint Positions')
    else:
        raise ValueError(f"Invalid set value: {set}. Must be 1-4.")
    
    # Plot selected 7 joints from joint_ids
    for i, joint_id in enumerate(joint_ids[joint_slice]):
        joint_hist = get_joint_hist(joints_hist_raw, joint_id)
        subplot_joint_pos_hist(robot, joint_hist, joint_id, axs[i])
    
    plt.tight_layout()
    plt.show()
    
    return

def plot_all_limb_joints(robot, joints_hist_raw):
    """Plot joint positions for all limbs in a 7x4 grid.
    
    Args:
        robot: PyBullet robot instance
        joints_hist_raw: Raw joint history data
    """
    # Create figure with 7x4 subplots grid
    fig_pos, axs_pos = plt.subplots(7, 4, figsize=(20, 20))
    fig_vel, axs_vel = plt.subplots(7, 4, figsize=(20, 20))
    fig_trq, axs_trq = plt.subplots(7, 4, figsize=(20, 20))

    joint_ids = get_joint_ids()
    
    # Plot all joints in a 7x4 grid
    for col in range(4):

        # Calculate slice for this column
        start_idx = col * 7
        end_idx = start_idx + 7
        joint_slice = slice(start_idx, end_idx)
        
        # Plot the 7 joints for this limb in the current column
        for row, joint_id in enumerate(joint_ids[joint_slice]):
            joint_hist = get_joint_hist(joints_hist_raw, joint_id)
            subplot_joint_pos_hist(robot, joint_hist, joint_id, axs_pos[row, col])
            subplot_joint_vel_hist(robot, joint_hist, joint_id, axs_vel[row, col])
            subplot_joint_trq_hist(robot, joint_hist, joint_id, axs_trq[row, col])
            
    # add tick labels to the bottom row 
    for col in range(4):
        axs_pos[-1, col].set_xlabel('Time (s)')
        axs_vel[-1, col].set_xlabel('Time (s)')
        axs_trq[-1, col].set_xlabel('Time (s)')

    for row in range(7):
        axs_pos[row, 0].set_ylabel('Pos (rad)')
        axs_vel[row, 0].set_ylabel('Vel (rad/s)')
        axs_trq[row, 0].set_ylabel('Trq (Nm)')

    fig_pos.suptitle('Crab Joint Positions') 
    fig_vel.suptitle('Crab Joint Velocities') 
    fig_trq.suptitle('Crab Joint Torques') 

    plt.show()
    
    return 

# plot the joint position history for all joints 
def plot_joint_pos_hist_all(robot, joints_hist_raw):

    for joint_id in get_joint_ids():
        joint_hist = get_joint_hist(joints_hist_raw, joint_id) 
        plot_joint_pos_hist(robot, joint_hist, joint_id) 

# ---------------------------------- 
# ---------------------------------- 


if __name__ == "__main__":


    # load the pickle file 
    pickle_path = os.path.join(cwd, 'test/crab/', 'joints_hist_raw.pkl')
    with open(pickle_path, 'rb') as f:
        joints_hist_raw = pickle.load(f)

    # Connect to PyBullet
    pb.connect(pb.DIRECT)  # Use DIRECT mode since we don't need visualization

    # Load robot
    robot = pb.loadURDF(
        cwd + "/robot_model/crab/crab.urdf",
        [0, 0, 0],
        [0, 0, 0, 1],
        useFixedBase=False,
    )
    
    # Plot all joints
    plot_all_limb_joints(robot, joints_hist_raw)

    # Disconnect from PyBullet
    pb.disconnect()

