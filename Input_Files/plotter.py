import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D 
import matplotlib
import os
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline
import pandas as pd

plt.rcParams["grid.color"] = "lightgray"
plt.rcParams["grid.linewidth"] = 0.5
matplotlib.rc("font", family="serif", size=7)
matplotlib.rcParams["text.usetex"] = True

def main():
    # Read data from csv file
    filename = "actuation-miltisine.csv";
    trajectory = np.genfromtxt(os.path.dirname(__file__) + "/" + filename, delimiter=",", dtype=float, skip_header=1)

    # Separate the columns into x, y, and z
    t = trajectory[:,0];
    q = trajectory[:,1];      # insertion, rotation, anterior, right, posterior, left ()
    q_dot = trajectory[:,2];

    # Create a 3D scatter plot
    column = 2;
    fig, axs = plt.subplots(2, 1, figsize=(14, 7))

    axs[0].set_title("Tip position")
    axs[0].plot(t, q*1E3, c="black", label="X", linewidth=1.0, linestyle="-")  # Scatter plot with red circles
    axs[0].set_ylabel("$\mathrm{tip}$ [mm]")
    axs[0].minorticks_on()
    axs[0].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
    axs[0].legend()

    axs[1].set_title("Tendon position - Positive is pulling")
    # ax[1].plot(t, q[:, column]*1e3, c="black", label="q", linewidth=1.0)  # Scatter plot with red circles
    axs[1].plot(t, q_dot*1e3, c="black", label="pos", linewidth=1.0, linestyle="-")  # Scatter plot with red circles
    # ax[1].plot(resample_t, joint_pos_interp[:, column]*1e3, c="red", label="pos_resample", linewidth=1.5, linestyle="--")  # Scatter plot with red circles
    # ax[1].plot(t, joint_pos_targ[:, column]*1e3, c="red", label="ref", linewidth=1.5, linestyle=":")  # Scatter plot with red circles
    axs[1].set_ylabel("$\mathrm{joint}$ [mm]")
    axs[1].minorticks_on()
    axs[1].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
    axs[1].legend(loc="best")


    plt.subplots_adjust(hspace=0.3)  # Set the spacing between subplots
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(__file__), filename + '.pdf'), format='pdf')
    plt.show(block=True)
    

if __name__ == "__main__":
    main()
