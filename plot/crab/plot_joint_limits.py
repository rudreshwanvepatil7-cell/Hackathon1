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

    joint_pos = joint_hist['pos']
    joint_vel = joint_hist['vel'] 
    joint_torque = joint_hist['torque']
    sim_time = joint_hist['sim_time']

    # Get joint name from joint info
    joint_data = get_joint_data(robot, joint_id) 
    joint_name = joint_data['joint_name']

    joint_limits = get_joint_limits(robot, joint_id)

    ax.plot(sim_time, joint_pos)
    # ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pos (rad)')
    
    # Add horizontal lines for joint limits
    lower_limit, upper_limit = joint_limits
    ax.axhline(y=upper_limit, color='r', linestyle='--', label='Upper Limit')
    ax.axhline(y=lower_limit, color='r', linestyle='--', label='Lower Limit')
    # ax.legend()

    ax.text(0.02, 0.90, joint_name, transform=ax.transAxes, 
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'),
            verticalalignment='top')

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

# plot the joint position history for all joints 
def plot_joint_pos_hist_all(robot, joints_hist_raw):

    for joint_id in get_joint_ids():
        joint_hist = get_joint_hist(joints_hist_raw, joint_id) 
        plot_joint_pos_hist(robot, joint_hist, joint_id)

# ---------------------------------- 
# ---------------------------------- 


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_file', type=str, required=True,
                      help='Path to the pickle file containing joint data')
    args = parser.parse_args()

    # Load data
    with open(args.data_file, 'rb') as f:
        joints_hist = pickle.load(f)
    
    # Plot all joints
    plot_all_joint_positions(robot, joints_hist)

