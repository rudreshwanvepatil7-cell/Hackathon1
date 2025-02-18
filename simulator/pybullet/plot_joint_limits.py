import numpy as np
import matplotlib.pyplot as plt
import pickle
import argparse
from datetime import datetime

def main():

    # load the pickle file
    with open('joint_pos_data_all_hist.pkl', 'rb') as f:
        joint_pos_data_all_hist = pickle.load(f)

    # plot the joint position at the first time instance and for 1st joint 
    sim_time = joint_pos_data_all_hist[0]['sim_time'] 
    joint_pos_data_all = joint_pos_data_all_hist[0]['joint_pos_data_all']

    # plot the joint position at the first time instance and for 1st joint 
    plt.plot(sim_time, joint_pos_data_all[0]['position'])
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Position (rad)')
    plt.title('Joint Position vs Time')
    plt.show()

if __name__ == "__main__":
    main() 