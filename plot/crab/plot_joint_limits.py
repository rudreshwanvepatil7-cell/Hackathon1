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

def plot_joint_position_history(robot, joint_hist, joint_id ):

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

if __name__ == "__main__":
    main() 

