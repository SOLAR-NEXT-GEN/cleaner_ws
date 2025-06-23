#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from collections import deque

class VelocityPlotter(Node):
    def __init__(self):
        super().__init__('velocity_plotter')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        self.vl_data = deque(maxlen=200)
        self.vr_data = deque(maxlen=200)
        self.time_data = deque(maxlen=200)
        self.counter = 0

        # Setup the plot only once
        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], label='v_l', color='blue')
        self.line2, = self.ax.plot([], [], label='v_r', color='orange')
        self.ax.set_xlabel('Time Steps')
        self.ax.set_ylabel('Velocity (m/s)')
        self.ax.set_title('Real-Time Velocity Plot')
        self.ax.legend()
        self.fig.canvas.manager.set_window_title('Velocity Plot')

        # Create the timer to update the plot
        self.create_timer(0.1, self.timer_callback)

    def listener_callback(self, msg):
        if len(msg.data) == 2:
            vl, vr = msg.data
            self.vl_data.append(vl)
            self.vr_data.append(vr)
            self.time_data.append(self.counter)
            self.counter += 1

    def timer_callback(self):
        if not self.time_data:
            return

        self.line1.set_data(self.time_data, self.vl_data)
        self.line2.set_data(self.time_data, self.vr_data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    plt.ion()  # Turn on interactive mode
    node = VelocityPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
