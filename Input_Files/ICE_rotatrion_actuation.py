import numpy as np
import matplotlib.pyplot as plt
import csv, os
import pandas as pd

# Parameters
omega = 2 * np.pi * 0.2  # Angular frequency in rad/s (e.g., 0.1 Hz)
A = 6.00E-3  # Amplitude of the rotation
k = 7 # Steepness of the sigmoid
t_c = 2 #Midpoint of the sigmoid curve
duration = 100.00  # Duration of the simulation in seconds
dt = 4.00E-3  # Time step
sampling_rate = 1 / dt  # Sampling rate in Hz
time = np.linspace(0, duration, int(duration * sampling_rate))  # Time vector
sigmoid = 1 / (1 + np.exp(-k * (time - t_c)))

# Tendon actuation model (assuming a simple sinusoidal pattern)
tendon1 = sigmoid * A * np.sin(omega * time)  # Tendon 1 actuation (pulling in one direction)
tendon2 = sigmoid * A * np.cos(omega * time)  # Tendon 2 actuation (pulling in one direction)
tendon3 = sigmoid * -A * np.sin(omega * time)  # Tendon 3 actuation (pulling in the opposite direction)
tendon4 = sigmoid * -A * np.cos(omega * time)  # Tendon 4 actuation (pulling in the opposite direction)

# Clamp all negative values in the tendon variables to zero
tendon1 = np.maximum(tendon1, 0)
tendon2 = np.maximum(tendon2, 0)
tendon3 = np.maximum(tendon3, 0)
tendon4 = np.maximum(tendon4, 0)

# Create a matrix to store the actuation values (time, tendon1, tendon2, tendon3, tendon4)
actuation_data = np.column_stack((time, tendon1, tendon2, tendon3, tendon4))

# Define the CSV file name
csv_filename = 'ICE_rotational_Actuation.csv'
csv_path = os.path.join(os.path.dirname(__file__), csv_filename)

trajectory_df = pd.DataFrame(actuation_data, columns=['Time (s)', 'Tendon 1', 'Tendon 2', 'Tendon 3', 'Tendon 4'])
trajectory_df.to_csv(csv_path, index=False, float_format="%.6f")

print(f'CSV file "{csv_filename}" has been generated successfully.')

# Plot the tendon actuation values
plt.figure()
plt.plot(time, tendon1, 'r', label='Tendon 1')
plt.plot(time, tendon2, 'g', label='Tendon 2')
plt.plot(time, tendon3, 'b', label='Tendon 3')
plt.plot(time, tendon4, 'k', label='Tendon 4')
plt.xlabel('Time (s)')
plt.ylabel('Tendon Actuation')
plt.legend()
plt.title('Tendon Actuation Over Time')
plt.grid(True)
plt.show()
