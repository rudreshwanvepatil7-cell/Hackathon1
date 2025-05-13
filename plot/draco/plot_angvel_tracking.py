import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

sns.set_theme(style="darkgrid", font="serif")

xdf = pd.read_csv("experiment_data/xdata.csv")
ydf = pd.read_csv("experiment_data/ydata.csv")
xydf = pd.read_csv("experiment_data/xydata.csv")


def make_plot(data: pd.DataFrame, xkey: str, title: str, xlabel: str):
    data = data.melt(xkey, var_name="variation", value_name="max_yaw")
    ax = sns.pointplot(x=xkey, y="max_yaw", hue="variation", data=data)
    ax.set_title(title)
    ax.set(xlabel=xlabel, ylabel="maximum yaw rate (rad/s)")
    ax.legend(title="Controller", loc="upper right")
    plt.savefig(f"experiment_data/{xkey}_plot.png", dpi=300)
    plt.close()


make_plot(xdf, "x", "Turn Speeds for Forward (x) Velocity Tracking", "x velocity (m/s)")
make_plot(ydf, "y", "Turn Speeds for Lateral (y) Velocity Tracking", "y velocity (m/s)")
make_plot(xydf, "xy", "Turn Speeds for Diagonal (x-y) Velocity Tracking", "x-y velocity (m/s)")
