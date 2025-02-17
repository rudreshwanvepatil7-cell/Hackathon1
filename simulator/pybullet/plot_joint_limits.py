import numpy as np
import matplotlib.pyplot as plt
import pickle
import argparse
from datetime import datetime

def plot_joint_limits(time, joint_positions, joint_limits, joint_names):
    """
    Plot joint positions and their limits
    Args:
        time: time vector
        joint_positions: Array of joint positions over time (n_timesteps x n_joints)
        joint_limits: Dictionary with 'upper' and 'lower' limits for each joint
        joint_names: List of joint names
    """
    n_joints = len(joint_names)
    fig, axes = plt.subplots(n_joints, 1, figsize=(10, 2*n_joints), sharex=True)
    
    for i, (joint, ax) in enumerate(zip(joint_names, axes)):
        # Plot joint trajectory
        ax.plot(time, joint_positions[:, i], 'b-', label='Position', linewidth=2)
        
        # Plot limits as horizontal lines
        ax.axhline(y=joint_limits['upper'][i], color='r', linestyle='--', 
                  label='Upper Limit')
        ax.axhline(y=joint_limits['lower'][i], color='r', linestyle='--', 
                  label='Lower Limit')
        
        # Highlight regions where joint is near limits
        margin = 0.05  # 5% margin from limits
        limit_range = joint_limits['upper'][i] - joint_limits['lower'][i]
        upper_threshold = joint_limits['upper'][i] - margin * limit_range
        lower_threshold = joint_limits['lower'][i] + margin * limit_range
        
        # Shade regions where joint is close to limits
        ax.fill_between(time, 
                       joint_limits['upper'][i], 
                       upper_threshold,
                       color='red', alpha=0.2)
        ax.fill_between(time, 
                       joint_limits['lower'][i],
                       lower_threshold, 
                       color='red', alpha=0.2)
        
        ax.set_ylabel(f'{joint} (rad)')
        ax.grid(True)
        if i == 0:
            ax.legend()

    axes[-1].set_xlabel('Time (s)')
    plt.suptitle('Joint Positions and Limits')
    plt.tight_layout()
    return fig

def save_joint_data(data_dict, filename):
    with open(filename, 'wb') as f:
        pickle.dump(data_dict, f)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_file', type=str, default='experiment_data/pnc.pkl',
                      help='Path to the pickle file containing joint data')
    args = parser.parse_args()

    # Load data
    time = []
    joint_positions = []
    
    with open(args.data_file, 'rb') as file:
        while True:
            try:
                d = pickle.load(file)
                time.append(d['time'])
                joint_positions.append(d['joint_positions'])
            except EOFError:
                break
    
    joint_positions = np.stack(joint_positions, axis=0)
    
    # # Define joint limits (replace with actual limits)
    # joint_names = ['joint1', 'joint2', 'joint3']  # Replace with actual joint names
    # joint_limits = {
    #     'upper': np.array([2.0, 2.0, 2.0]),  # Replace with actual upper limits
    #     'lower': np.array([-2.0, -2.0, -2.0])  # Replace with actual lower limits
    # }

    # Define joint names for the crab robot
    joint_names = [
        'back_left_hip_roll', 'back_left_hip_pitch', 'back_left_knee',
        'back_right_hip_roll', 'back_right_hip_pitch', 'back_right_knee',
        'front_left_hip_roll', 'front_left_hip_pitch', 'front_left_knee',
        'front_right_hip_roll', 'front_right_hip_pitch', 'front_right_knee'
    ]

    # Define joint limits (in radians)
    joint_limits = {
        'upper': np.array([2.0, 1.57, 2.0] * 4),  # Adjust these values based on actual robot limits
        'lower': np.array([-2.0, -1.57, -2.0] * 4)
    }
    
    # Create plot
    fig = plot_joint_limits(time, joint_positions, joint_limits, joint_names)
    plt.show()

if __name__ == "__main__":
    main()