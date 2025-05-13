import sys
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.signal import lfilter
from typing import Optional
import seaborn as sns
import pandas as pd

cwd = os.getcwd()
sys.path.append(cwd)

from plot.helper import plot_task, load_pkl_data, plot_vector_traj

X = np.array([0, 0.2, 0.5, 0.7, 1.0, 1.2])
Y = np.array([0, 0.2, 0.5, 0.7, 1.0, 1.1])
XY = np.array([0, 0.2, 0.5, 0.7, 0.9])

TRACKING_X_NOLEAN = np.array([2.0, 2.0, 1.6, 1.4, 0.8, 0.7])


def moving_average(data, window_size):
    cumsum = np.cumsum(np.insert(data, 0, np.zeros(data.shape[1:]), axis=0), axis=0)
    return (cumsum[window_size:] - cumsum[:-window_size]) / window_size


def extract_by_key(data: list[dict], key: str, *, default=0.0) -> list:
    return np.array(list(map(lambda row: row.get(key, default), data)))


def extract_xyz_by_key(data: list[dict], key: str, *, default={"x": 0.0, "y": 0.0, "z": 0.0}) -> list:
    data = extract_by_key(data, key, default=default)
    return np.array(list(map(lambda row: [row["x"], row["y"], row["z"]], data)))


def plot_vel_task(time, vel_des, vel, suptitle, label=None, maxtime: Optional[float] = None, avg_window: int = 1):
    if maxtime:
        vel = vel[time < maxtime]
        vel_des = vel_des[time < maxtime]
        time = time[time < maxtime]
    time = time[avg_window - 1 :]
    vel_des = vel_des[avg_window - 1 :]
    vel = moving_average(vel, avg_window)

    dim = vel_des.shape[1]
    fig, axes = plt.subplots(dim)
    for i in range(dim):
        axes[i].plot(time, vel_des[:, i], color="r", linestyle="dashed", linewidth=4)
        axes[i].plot(time, vel[:, i], color="b", linewidth=2)
        axes[i].grid(True)
        if label is not None:
            axes[i].set_ylabel(label[i])
    axes[dim - 1].set_xlabel("time")
    fig.suptitle(suptitle)


def movingaverage(interval, window_size):
    window = np.ones(int(window_size)) / float(window_size)
    return np.convolve(interval, window, "same")


def plot_velocity_tracking(data):
    time = extract_by_key(data, "time")
    # command
    linvel_des = extract_xyz_by_key(data, "vel_cmd")
    yaw_vel_des = extract_by_key(data, "yaw_rate_cmd")[:, np.newaxis]
    vel_des = np.hstack((linvel_des[:, :2], yaw_vel_des))
    # actual
    linvel = extract_by_key(data, "com_lin_vel", default=[0.0, 0.0, 0.0])[:, :2]
    angvel = extract_by_key(data, "com_ang_vel", default=[0.0, 0.0, 0.0])[:, 2:]
    vel = np.concatenate([linvel, angvel], axis=-1)
    plot_vel_task(time, vel_des, vel, "test", label=["xdot", "ydot", "yawdot"], maxtime=100, avg_window=5)


def plot_grfs(data):
    time = extract_by_key(data, "time")
    lfoot_rf = np.linalg.norm(extract_by_key(data, "lfoot_rf_cmd"), axis=-1)
    rfoot_rf = np.linalg.norm(extract_by_key(data, "rfoot_rf_cmd"), axis=-1)
    df = pd.DataFrame({"time": time, "lfoot_rf": lfoot_rf, "rfoot_rf": rfoot_rf})
    footcols = ["lfoot_rf", "rfoot_rf"]
    df[footcols] = df[df[footcols] > 1.0][footcols]
    df.dropna()
    sns.lineplot(x="time", y="lfoot_rf", data=df)


if __name__ == "__main__":
    data = load_pkl_data()
    # plot_grfs(data)
    plot_velocity_tracking(data)
    # plot_orientation_tracking(data)
    # axes = plot_vector_traj(time, pos, phase, None, None)  # orientation tracking
    plt.show()
