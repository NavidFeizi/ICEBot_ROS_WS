import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import random, os

# Read the input CSV file containing possible target values
input_csv = "possible_task_targets.csv"  # Replace with your input CSV file
output_csv = "pseudo_random_trajectory.csv"  # Output CSV file

# Load the input data
targets_df = pd.read_csv(os.path.dirname(__file__) + "/" + input_csv)

# Configuration parameters
sample_time = 0.004  # Sample time in seconds
total_duration = 30.0  # Total duration of the trajectory in seconds
min_step_duration = 0.5  # Minimum step duration in seconds
max_step_duration = 2.0  # Maximum step duration in seconds

# Initialize variables
time = 0.0
trajectory_data = []

# Generate the trajectory
while time < total_duration:
    # Randomly select a target from the input CSV
    random_target = targets_df.sample(n=1).iloc[0]
    p_targ_x = random_target['p_targ_x']
    p_targ_z = random_target['p_targ_z']
    
    # Randomly determine the duration of this step
    step_duration = random.uniform(min_step_duration, max_step_duration)
    num_samples = int(step_duration / sample_time)
    
    # Generate samples for the current step
    for _ in range(num_samples):
        trajectory_data.append([time, p_targ_x, 0.0, p_targ_z, 0.0])
        time += sample_time
        if time >= total_duration:
            break

trajectory_df = pd.DataFrame(trajectory_data, columns=["time", "p_targ_x", "v_targ_x", "p_targ_z", "v_targ_z"])
trajectory_df.to_csv(output_csv, index=False, float_format="%.6f")


fig, axs = plt.subplots(4, 1, figsize=(14, 7))
axs[0].plot(trajectory_df["time"], trajectory_df["p_targ_x"], c="black", label="P_x", linewidth=1.0, linestyle="-")  # Scatter plot with red circles
axs[0].set_ylabel("$P_x$ [m]")
axs[0].minorticks_on()
axs[0].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
axs[0].legend()

axs[1].plot(trajectory_df["time"], trajectory_df["v_targ_x"], c="black", label="V_x", linewidth=1.0, linestyle="-")  # Scatter plot with red circles
axs[1].set_ylabel("$V_x$ [m]")
axs[1].minorticks_on()
axs[1].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
axs[1].legend()

axs[2].plot(trajectory_df["time"], trajectory_df["p_targ_z"], c="black", label="P_z", linewidth=1.0, linestyle="-")  # Scatter plot with red circles
axs[2].set_ylabel("$P_z$ [m]")
axs[2].minorticks_on()
axs[2].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
axs[2].legend()

axs[3].plot(trajectory_df["time"], trajectory_df["v_targ_z"], c="black", label="V_z", linewidth=1.0, linestyle="-")  # Scatter plot with red circles
axs[3].set_ylabel("$V_z$ [m]")
axs[3].minorticks_on()
axs[3].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
axs[3].legend()

plt.subplots_adjust(hspace=0.3)  # Set the spacing between subplots
plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(__file__), "pseudo_random_trajectory" + '.pdf'), format='pdf')
plt.show(block=True)

