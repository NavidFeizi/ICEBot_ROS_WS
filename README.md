<div align="center">

# Koopman-Controlled Steerable Catheter ROS Workspace

</div>

All nodes are coded in C++ with efficient calculation. This set of packages is all for simulation and experiment. Performance is tested and validated.


## Introduction
Packages:

* [robot](./src/robot/README.md)
* [emtracker](./src/emtracker/README.md)
* [controller](./src/controller/README.md)
* [catheter_sim_koopman](./src/catheter_sim_koopman/README.md)
* [publisher](./src/publisher/README.md)

## Building Requirements

ensure that you have the following libraries installed in your system:

* [Boost](https://www.boost.org/)
* [Blaze Library](https://bitbucket.org/blaze-lib/blaze/src/master/)
* [LAPACK](http://www.netlib.org/lapack/)
* [NLopt](https://nlopt.readthedocs.io/en/latest/)

* ROS2 jazzy

* 
   ```bash
   sudo apt install ros-jazzy-plotjuggler
   sudo apt install ros-jazzy-plotjuggler-ros
   ```

## Build Packages Instruction

To build the nodes, follow these instructions:

1. **Build the packages:**
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select interfaces manager emtracker robot
   ```

2. **Common build issues:**
   - If you encounter issues related to linking `libtorch` and `rclcpp`, ensure you download the cxx11 ABI version of `libtorch` from pytorch.org.
   - You may need to add `libtorch_cpu.so` to the system environment:
     ```bash
     export LD_LIBRARY_PATH=/usr/local/libtorch/lib:$LD_LIBRARY_PATH
     ```


## Setup the Hardware

### Setup CANopen Connection to the Robot

To set up the CANopen connection, follow these steps:

1. **Load the CANopen driver:**
   ```bash
   modprobe ix_usb_can
   ```
   If you encounter an error, the IXXAT socket CAN driver is not installed. Install the compatible version with your Linux kernel (IXXAT_SocketCAN_2_0_378_Modified_2023-03-15).

2. **Configure CAN interface:**
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   sudo ip link set can0 txqueuelen 1000
   sudo ip link set can0 down
   sudo ip link set can0 type can loopback on
   sudo ip link set can0 up
   ```
3. **Monitor CAN connection:**
   Open a terminal and run:
   ```bash
   candump can0
   ```

4. **Send reset command to check all nodes:**
   Open a separate terminal and run:
   ```bash
   cansend can0 000#8200
   ```
   Among the return packets if you see a line similar to the one below, which is the echo of the reset command, you are good to go. Otherwise, the "loopback" setting might not be on. Sometimes, you need to replug the USB port to fix it.
   ```plaintext
   can0  000   [2]  82 00
   ```

### Setup EMtracker USB Connection

1. **Identify connected USB ports:**
   ```bash
   ls /dev/tty*
   ```

2. **Grant access permission to the port:**
   ```bash
   sudo chmod a+rw /dev/ttyUSB*
   ```
   Replace `*` with the number of the connected USB port, usually `0`.




## Running the Nodes

1. **Isolate CPU cores:**

   First isolate CPU cores through GRUB config. Open the GRUB configuration file:

   ```bash
   sudo nano /etc/default/grub
   ```
   Find the line starting with `GRUB_CMDLINE_LINUX_DEFAULT`, and add the following kernel parameters as needed:

   `isolcpus=0,1`: Isolates cores 0 and 1 from normal OS scheduling.

   `nohz_full=0,1`: Enables "no tick" mode, which reduces kernel interruptions on these cores.

   `rcu_nocbs=0,1`: Moves RCU (Read-Copy-Update) callbacks off these cores to prevent kernel delays.

   ```bash
   GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=0,1 nohz_full=0,1 rcu_nocbs=0,1"
   ```

2. **Run on specific cores:**
   ```bash
   taskset -c 0,1 ros2 run catheter_sim_koopman_cpp simulator
   ```
   Note: `taskset` can also be included in the launch file.

3. **Triger robot homing service:**
   ```bash
   ros2 service call /homing std_srvs/srv/Trigger
   ```

4. **Set parameters:**
   ```bash
   ros2 param set control_node Q "[0.7, 0.0, 0.0, 0.0]"
   ros2 param set control_node R "[5.0e-4]"
   ros2 param set control_node cutoff_freq 2.0
   ros2 param set observer_node Q "[0.01, 0.01]"
   ros2 param set emtracker_node cutoff_freq 1.0
   ros2 param set robot_node Kp 4.6
   ```

## Troubleshooting

- **Error:** `/usr/bin/ld: cannot find -lgfortran: No such file or directory`
  - Solution: Install the correct version of `gfortran`. For example:
    ```bash
    sudo apt install gcc-11 g++-11 gfortran-11
    ```
    or
    ```bash
    sudo apt install gcc-12 g++-12 gfortran-12
    ```
    Ensure the `gfortran` version aligns with the default `gcc` in the system.


## Current Problems
There is an error in the model velocity; it does not align with the model position. When the position is stationary, the velocity is maximum! Check the model to ensure it is correct.

The Extended Kalman Filter (EKF) is very sensitive to the measurement. Reducing the measurement noise covariance to improve state prediction destabilizes the system. The current values in the observer launch.py are the optimal ones I could find. Also, reducing [R] causes the amplitude of the estimated velocity to become too high compared to the actual velocity. I also notice sensitivity to the sampling time and the number of iterations. I believe this is due to the discrepancy between the model and the experiment.

What I can investigate: Add some phase lag or delay to the measurement signal in simulation and investigate if you see the increase in the velocity amplitude as well. Improve the model to reduce uncertainties (retrain and fine-tune for the actual setup). Find a method for adjusting the pretension of the tendon; this does have a significant effect on the system and needs to be balanced. Look for and try an unscented Kalman Filter that handles nonlinear systems better.

