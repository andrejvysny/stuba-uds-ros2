from math import sqrt, pow
import matplotlib.pyplot as plt
import rclpy.time
from nav_msgs.msg import Odometry
from rclpy.node import Node
import rclpy
from pynput import keyboard
from time import time
from datetime import datetime

class Plot(Node):

    """
    The Plot class is a ROS node that subscribes to the 'odom' topic and plots the total distance travelled by the robot over time.
    """
    def __init__(self):
        """
        Initialize the Plot node.
        """
        super().__init__('plot')
        self.x = 0.0
        self.y = 0.0
        self.subscriber = self.create_subscription(Odometry, 'odom', self.plot_callback, 10)
        self.total_distance = 0.0
        self.old_x = 0.0
        self.old_y = 0.0
        self.time_list = []
        self.distance_list = []
        self.last_time = time()


    def plot_callback(self, msg):
        """
        Callback function for the 'odom' topic subscription. Updates the total distance travelled and the time and distance lists for plotting.
        """
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        # Only add to total distance if this is not the first message
        if self.old_x is not None and self.old_y is not None:
            self.total_distance += sqrt(pow(new_x - self.old_x, 2) + pow(new_y - self.old_y, 2))
        self.old_x = new_x
        self.old_y = new_y
        print(f"Total distance: {self.total_distance}")
        current_time = time()
        if current_time - self.last_time >= 0.05:  # 50 ms have passed
            real_time = datetime.fromtimestamp(current_time)
            self.time_list.append(real_time)
            self.distance_list.append(self.total_distance)
            self.last_time = current_time

should_shutdown = False

def on_release(key):
    """
    Callback function for keyboard release events. Shuts down the node if the escape key is released.
    """
    global should_shutdown
    try:
        if key == keyboard.Key.esc:
            print("Shutting down...")
            should_shutdown = True
    except AttributeError:
        print("An error occurred while processing the key release.")

def main(args=None):
    """
    Main function. Initializes the node, starts the keyboard listener, spins the node, plots the graph, and shuts down the node.
    """
    rclpy.init(args=args)
    plot = Plot()

    listener = keyboard.Listener(on_release=on_release)
    listener.start()

    while not should_shutdown:
        rclpy.spin_once(plot, timeout_sec=0.1)

    # Plot the graph after spinning
    plt.plot(plot.time_list, plot.distance_list)
    plt.xlabel('Time (s)')
    plt.ylabel('Total Distance (m)')
    plt.title('Total Distance Over Time')
    plt.show()
    
    plot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()