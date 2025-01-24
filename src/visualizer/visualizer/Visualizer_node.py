import rclpy
from rclpy.node import Node

from interfaces.msg import Tippos 
from interfaces.msg import Forcetorque 
from interfaces.msg import Jointsstatus 
from interfaces.msg import Jointstarget 

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class Visualizer(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription_1 = self.create_subscription(Forcetorque, 'FTsensor', self.listener_callback_1, 1)
        # self.subscription_1 = self.create_subscription(Tippos, 'EMTTipSpace', self.listener_callback_1, 1)
        # self.subscription_2 = self.create_subscription(Robotconfig, 'TargetJointSpace', self.listener_callback_2, 1)
        # self.subscription_1  # prevent unused variable warning

        self.timer = self.create_timer(0.01, self.plot_data)  # Update plot every 0.1 seconds

        self.force = np.array([0.00, 0.00, 0.00])
        self.torque = np.array([0.00, 0.00, 0.00])
        self.t0 = self.get_clock().now().nanoseconds

        self.fig, self.ax = plt.subplots(2, 1, figsize=(8, 6))

         # Initialize lists to store historical data
        self.max_samples = 1000
        self.force_history = [[0.0] * self.max_samples for _ in range(3)]
        self.torque_history = [[0.0] * self.max_samples for _ in range(3)]
        self.sample_count = 0

    def listener_callback_1(self, msg):
        self.force[0] = msg.force[0];
        self.force[1] = msg.force[1];
        self.force[2] = msg.force[2];
        self.torque[0] = msg.torque[0];
        self.torque[1] = msg.torque[1];
        self.torque[2] = msg.torque[2];
    
        # Append new force and torque values to historical lists
        for i in range(3):
            self.force_history[i].append(msg.force[i])
            self.torque_history[i].append(msg.torque[i])

            # Maintain a maximum of 2000 samples in the historical data
            if len(self.force_history[i]) > self.max_samples:
                self.force_history[i] = self.force_history[i][-self.max_samples:]
                self.torque_history[i] = self.torque_history[i][-self.max_samples:]

        # Update sample count
        self.sample_count += 1

        force_str = ",  ".join([f"{force:.5f}" for force in msg.force])
        torque_str = ",  ".join([f"{torque:.5f}" for torque in msg.torque])
        print(f"Force: [{force_str}]   Torque: [{torque_str}]")


        
    # def listener_callback_3(self, msg):
    #     t = (self.get_clock().now().nanoseconds-self.t0)/1e9
    #     self.data = np.append(self.data, np.array([[t, self.data.shape[0], msg.target_xyz[0], msg.target_xyz[1], msg.target_xyz[2], self.joint[0], self.joint[1], self.joint[2], self.joint[3], self.joint[4], self.joint[5]]]), axis=0)
    #     self.get_logger().info('time: %0.3f    Sample: %d   X: %0.2f    Y: %0.2f    Z: %0.2f' %(t, self.data.shape[0], msg.target_xyz[0], msg.target_xyz[1], msg.target_xyz[2]))
    #     self.get_logger().info('L1: %0.2f    L2: %0.2f  L3: %0.2f    R1: %0.2f    R2: %0.2f    R3: %0.2f' %(self.joint[0], self.joint[1], self.joint[2], self.joint[3], self.joint[4], self.joint[5]))
    #     # self.get_logger().info('Shape: %d  %d' %(self.data.shape[0], self.data.shape[1]))

    # def save(self):
    #     self.get_logger().info('Saving...')
    #     data_df = pd.DataFrame({"Time": self.data[:, 0], "Sample": self.data[:, 1], "X": self.data[:, 2], "Y": self.data[:, 3], "Z": self.data[:, 4], "L1": self.data[:, 5], "L2": self.data[:, 6], "L3": self.data[:, 7], "R1": self.data[:, 8], "R2": self.data[:, 9], "R3": self.data[:, 10]})
    #     data_df.to_csv('Recording.csv')
    #     self.get_logger().info('Recorde data saved!')

    def __del__(self):
        pass

    def plot_data(self):
        self.ax[0].cla()  # Clear previous plots
        self.ax[1].cla()

        for i in range(3):
            self.ax[0].plot(self.force_history[i], label=f'Force {i}')
            self.ax[1].plot(self.torque_history[i], label=f'Torque {i}')

        self.ax[0].legend()
        self.ax[0].set_title('Force Signals')
        self.ax[1].legend()
        self.ax[1].set_title('Torque Signals')

        plt.tight_layout()
        plt.draw()
        plt.pause(0.0001)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Visualizer()
    # rclpy.spin(minimal_subscriber)
    # try:
    rclpy.spin(minimal_subscriber)
    # except:
        # pass
        # minimal_subscriber.save()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
