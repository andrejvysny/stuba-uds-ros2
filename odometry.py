import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import time
import time
class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10
        )
        self.subscription
        self.current_position = None
        self.previous_position = None

        self.total_distance = 0.0

        self.last_time = 0
        self.distance_array = []

    def calculate_time_difference(self):
            current_time = time.time()
            time_difference = current_time - self.last_time
            self.last_time = current_time
            return time_difference

    def odometry_callback(self, msg):
        time_difference = self.calculate_time_difference()

        position = msg.pose.pose.position
        self.previous_position = self.current_position
        self.current_position = position

        distance = 0.0
        if self.previous_position is not None:
            distance = self.calculate_distance(self.previous_position, self.current_position)
            self.total_distance += distance

            self.distance_array.append(self.total_distance)


        plt.plot(self.total_distance)
        plt.xlabel('Time')
        plt.ylabel('Distance')
        plt.title('Live Distance Plot')
        plt.grid(True)
        plt.show()

    def calculate_distance(self, position1, position2):
        dx = position2.x - position1.x
        dy = position2.y - position1.y
        dz = position2.z - position1.z
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        return distance


def main(args=None):
    rclpy.init(args=args)
    odometry_subscriber = OdometrySubscriber()
    rclpy.spin(odometry_subscriber)
    odometry_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()