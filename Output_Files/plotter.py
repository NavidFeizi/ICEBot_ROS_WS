import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib import rc
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import ScalarFormatter
import os
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline
import pandas as pd

plt.rcParams["grid.color"] = "lightgray"
plt.rcParams["grid.linewidth"] = 0.5
matplotlib.rc("font", family="serif", size=7)
matplotlib.rcParams["text.usetex"] = True

folderName = "2025-02-03_13-50-29";

def main():
    # Read data from csv file
    output_file = os.path.join(os.path.dirname(__file__), folderName , "calibration_data.csv")
    robot_data = np.genfromtxt(os.path.join(os.path.dirname(__file__), folderName, "joint_space.csv"), delimiter=",", dtype=float, skip_header=1)
    FT_data = np.genfromtxt(os.path.join(os.path.dirname(__file__), folderName, "FTsensor.csv"), delimiter=",", dtype=float, skip_header=1)
    EMT_data = np.genfromtxt(os.path.join(os.path.dirname(__file__), folderName, "task_space.csv"), delimiter=",", dtype=float, skip_header=1)

    # Separate the columns into x, y, and z
    t_joints = robot_data[:,0];
    q = robot_data[:,1:7];      # insertion, rotation, anterior, right, posterior, left ()
    q_dot = robot_data[:,7:13];
    joint_pos = robot_data[:,13:19];
    joint_pos_targ = robot_data[:,37:43];
    joint_pos_abs = robot_data[:,19:25];
    joint_velocity = robot_data[:,25:31];
    joint_current = robot_data[:,31:37];

    t_task = EMT_data[:,0];
    tip_pos = EMT_data[:, 1:4] 
    tip_pos_ts = EMT_data[:, 0:4] 
    tip_pos_targ = EMT_data[:, 13:16] 

    # Force = FT_data[:, 1:4] 

    joints_sample_time = np.diff(t_joints)*1e3
    task_sample_time = np.diff(t_task)*1e3

    joint_pos_ts = np.concatenate((np.expand_dims(t_joints,1), joint_pos), axis = 1)
    dataset_duration = t_joints[-1] - t_joints[0]

    # ### filter and interpolate tip position ###
    # window_length = 5  # Example window length
    # polyorder = 3  # Example polynomial order
    # upsample_dt = 1e-2
    # # find the fist changing sample
    # for k in range(1, 16):
    #     if ((tip_pos[k, 1] - tip_pos[k-1, 1]) > 0 or (tip_pos[k, 1] - tip_pos[k-1, 1]) < 0):
    #         if k >= 3:
    #             k = k - 3
    #         break

    # # # downsample to 16ms (emtracker actual sample time)
    # # tip_pos_ts_d = tip_pos_ts[k::4, :]
    # tip_pos_ts_d = tip_pos_ts

    # # filter the task space
    # tip_pos_ts_filtered = np.zeros_like(tip_pos_ts_d)
    # for i in range(tip_pos_ts_d.shape[1]):
    #     tip_pos_ts_filtered[:, i] = savgol_filter(tip_pos_ts_d[:, i], window_length, polyorder)
    # # upsample with cubic inerpolation
    # resample_t = np.arange(0.0, dataset_duration, upsample_dt)
    # tip_pos_interp = resampler(resample_t, tip_pos_ts_filtered)
    # tip_pos_interp = tip_pos_interp[:,1:]
    # # upsample with cubic inerpolation
    # joint_pos_interp = resampler(resample_t, joint_pos_ts)
    # joint_pos_interp = joint_pos_interp[:,1:]


    # headers = ["t", "x", "y", "z", "p_1", "p_2", "p_3", "p_4"]
    # data = np.concatenate((np.expand_dims(resample_t,1), tip_pos_interp, joint_pos_interp[:,2:]), axis = 1)
    # data_df = pd.DataFrame(data, columns=headers)
    # data_df.to_csv(output_file, index=False, float_format='%.8f')

    fig = plt.figure(figsize=(14, 7))
    gs = GridSpec(4, 4, figure=fig, height_ratios=[1, 1, 1, 1])
    axs = []
    axs.append(fig.add_subplot(gs[0, 0:4]))
    axs.append(fig.add_subplot(gs[1, 0:4]))
    axs.append(fig.add_subplot(gs[2, 0:4]))
    axs.append(fig.add_subplot(gs[3, 0]))
    axs.append(fig.add_subplot(gs[3, 1]))
    # axs.append(fig.add_subplot(gs[3, 2]))

    axs[1].set_title("Tip position")
    axs[0].plot(t_task, tip_pos[:,0]*1E3, c="r", label="X", linewidth=1.0)  # Scatter plot with red circles
    axs[0].plot(t_task, tip_pos[:,1]*1E3, c="g", label="Y", linewidth=1.0)  # Scatter plot with red circles
    axs[0].plot(t_task, tip_pos[:,2]*1E3, c="b", label="Z", linewidth=1.0)  # Scatter plot with red circles
    # axs[0].plot(t_task, tip_pos_targ[:,0]*1E3, c="r", label="X", linewidth=1.5, linestyle="--")  # Scatter plot with red circles
    # axs[0].plot(t_task, tip_pos_targ[:,1]*1E3, c="b", label="Y", linewidth=1.5, linestyle="--")  # Scatter plot with red circles
    # axs[0].plot(t_task, tip_pos_targ[:,2]*1E3, c="g", label="Z", linewidth=1.5, linestyle="--")  # Scatter plot with red circles
    # axs[0].plot(resample_t, tip_pos_interp[:,0]*1E3, c="r", label="X", linewidth=1.5, linestyle="--")  # Scatter plot with red circles
    # axs[0].plot(resample_t, tip_pos_interp[:,1]*1E3, c="b", label="Y", linewidth=1.5, linestyle="--")  # Scatter plot with red circles
    # axs[0].plot(resample_t, tip_pos_interp[:,2]*1E3, c="g", label="Z", linewidth=1.5, linestyle="--")  # Scatter plot with red circles
    axs[0].set_ylabel("$\mathrm{tip}$ [mm]")
    axs[0].minorticks_on()
    axs[0].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
    axs[0].legend()

    axs[1].set_title("Tendon position - Positive is pulling")
    # axs[1].plot(t, q[:, column]*1e3, c="black", label="q", linewidth=1.0)  # Scatter plot with red circles
    axs[1].plot(t_joints, joint_pos[:, 2]*1e3, c="black", label="T_1", linewidth=1.0, linestyle="-")  # Scatter plot with red circles
    axs[1].plot(t_joints, joint_pos[:, 3]*1e3, c="blue", label="T_3", linewidth=1.0, linestyle="-")  # Scatter plot with red circles
    axs[1].plot(t_joints, joint_pos[:, 4]*1e3, c="red", label="T_2", linewidth=1.0, linestyle="-")  # Scatter plot with red circles
    axs[1].plot(t_joints, joint_pos[:, 5]*1e3, c="green", label="T_4", linewidth=1.0, linestyle="-")  # Scatter plot with red circles

    axs[1].plot(t_joints, q[:, 2]*1e3, c="black", label="T_{1t}", linewidth=0.75, linestyle="--")  # Scatter plot with red circles
    axs[1].plot(t_joints, q[:, 3]*1e3, c="blue", label="T_{3t}", linewidth=0.75, linestyle="--")  # Scatter plot with red circles
    axs[1].plot(t_joints, q[:, 4]*1e3, c="red", label="T_{2t}", linewidth=0.75, linestyle="--")  # Scatter plot with red circles
    axs[1].plot(t_joints, q[:, 5]*1e3, c="green", label="T_{4t}", linewidth=0.75, linestyle="--")  # Scatter plot with red circles

    # ax[1].plot(resample_t, joint_pos_interp[:, column]*1e3, c="red", label="pos_res", linewidth=1.5, linestyle="--")  # Scatter plot with red circles
    # ax[1].plot(t, joint_pos_targ[:, column]*1e3, c="red", label="ref", linewidth=1.5, linestyle=":")  # Scatter plot with red circles
    axs[1].set_ylabel("$\mathrm{joint}$ [mm]")
    axs[1].minorticks_on()
    axs[1].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
    axs[1].legend(loc="best")

    axs[2].set_title("Tendon velocity - Positive is pulling")
    # axs[2].plot(t, q_dot[:, column] * 1e3, c="cyan", label="$\dot{q}$", linewidth=1.0)  # Position plot
    axs[2].plot(t_joints, joint_velocity[:, 2] * 1e3, c="black", label="T_1", linewidth=1.0, linestyle="-")  # Position plot
    axs[2].plot(t_joints, joint_velocity[:, 3] * 1e3, c="blue", label="T_3", linewidth=1.0, linestyle="-")  # Position plot
    axs[2].plot(t_joints, joint_velocity[:, 4] * 1e3, c="red", label="T_2", linewidth=1.0, linestyle="-")  # Position plot
    axs[2].plot(t_joints, joint_velocity[:, 5] * 1e3, c="green", label="T_4", linewidth=1.0, linestyle="-")  # Position plot
    axs[2].set_ylabel("$\mathrm{joint}$ [mm/s]")
    axs[2].minorticks_on()
    axs[2].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
    axs[2].legend(loc="best")
    axs[2].set_xlabel("$\mathrm{Time}$ [s]")

    axs[3].set_title("Distribution of sample times [ms]")
    axs[3].boxplot([joints_sample_time], labels=['Joints Sample Time'], patch_artist=True, whis=[0, 100])
    axs[3].set_ylabel("Sample Time [ms]")
    axs[3].minorticks_on()
    axs[3].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
    axs[3].legend(loc="best")
    # axs[3].yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    # axs[3].ticklabel_format(style='plain', axis='y')  # Ensure non-scientific notation

    axs[4].set_title("Distribution of sample times [ms]")
    axs[4].boxplot([task_sample_time], labels=['Task Sample Time'], patch_artist=True, whis=[0, 100])
    axs[4].set_ylabel("Sample Time [ms]")
    axs[4].minorticks_on()
    axs[4].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
    axs[4].legend(loc="best")
    # axs[4].yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
    # axs[4].ticklabel_format(style='plain', axis='y')  # Ensure non-scientific notation


    # ax[2].plot(t, current[:,column]*1e3, c="b", label="Current", linewidth=1)  # Scatter plot with red circles
    # ax[2].set_ylabel(r"$\mathrm{current}$ [mA]")
    # ax[2].minorticks_on()
    # ax[2].grid(which="minor", color="lightgray", linestyle="--", linewidth=0.5)
    # ax[2].legend()

    # ax[3].plot(t, Force[:,2], c="b", label="Force", linewidth=1.5)  # Scatter plot with red circles
    # ax[3].set_ylabel("$\mathrm{force}$ [N]")
    # ax[3].minorticks_on()
    # ax[3].grid(which="minor", color="lightgray", linestyle="--", linewidth=0.5)
    # ax[3].legend()



    plt.subplots_adjust(hspace=0.3)  # Set the spacing between subplots
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(__file__), folderName, 'plot.pdf'), format='pdf')
    plt.show(block=True)


    # # Error Distribution Plots
    # plt.figure(figsize=(12, 6))

    # # Histogram
    # plt.subplot(1, 3, 1)
    # plt.hist(error_x, bins=30, color="blue", alpha=0.7, label="Error X")
    # plt.hist(error_y, bins=30, color="red", alpha=0.7, label="Error Y")
    # plt.hist(error_z, bins=30, color="green", alpha=0.7, label="Error Z")
    # plt.xlabel("Error [mm]")
    # plt.ylabel("Frequency")
    # plt.legend()
    # plt.title("Error Histograms")

    # # Boxplot
    # plt.subplot(1, 3, 2)
    # sns.boxplot(data=[error_x, error_y, error_z], palette=["blue", "red", "green"])
    # plt.xticks([0, 1, 2], ["Error X", "Error Y", "Error Z"])
    # plt.title("Error Boxplot")

    # # Violin plot
    # plt.subplot(1, 3, 3)
    # sns.violinplot(data=[error_x, error_y, error_z], palette=["blue", "red", "green"])
    # plt.xticks([0, 1, 2], ["Error X", "Error Y", "Error Z"])
    # plt.title("Error Violin Plot")

    # plt.tight_layout()
    # plt.show()

    # except FileNotFoundError:
    #     print("The file 'EM_Trajectory.dat' was not found.")
    # except ValueError:
    #     print("Error: Unable to parse the data. Please check the format in the file.")
    # except Exception as e:
    #     print(f"An error occurred: {e}")


def resampler(new_time, time_series):
    interpolated = np.zeros((new_time.shape[0], time_series.shape[1]))
    interpolated[:,0] = new_time
    for j in range(1, time_series.shape[1]):
        cubic_spline = CubicSpline(time_series[:,0], time_series[:,j])
        interpolated[:,j] = cubic_spline(interpolated[:,0])
    return interpolated


if __name__ == "__main__":
    main()
