import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import time


class TurtlebotOpenLoop(Node):
    def __init__(self):
        super().__init__('tb_openLoop')
        
        # Publisher to control the robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to get the robot's position from the /odom topic
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer for publishing velocity
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Variables to store odometry data
        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.time_data = []

        # Open-loop control parameters
        self.scenario = 2  # Change to 2 for the second scenario
        self.start_time = time.time()

        # Flag to stop after a certain time
        self.robot_moving = True

    def timer_callback(self):
        elapsed_time = time.time() - self.start_time

        if not self.robot_moving:
            # Stop moving, do not publish velocity
            return

        msg = Twist()

        if self.scenario == 1:
            # Scenario 1: Constant velocity
            distance = 1.0  # meters
            time_to_travel = 10.0  # seconds
            velocity = distance / time_to_travel

            if elapsed_time < time_to_travel:
                msg.linear.x = velocity

            else:
                msg.linear.x = 0.0
                self.robot_moving = False  # Stop moving the robot
                self.plot_odometry_data()  # Plot and save after stopping

        elif self.scenario == 2:
            # Scenario 2: Acceleration -> Constant velocity -> Deceleration
            total_distance = 1.0  # meters
            accel_time = 2.0      # seconds for acceleration phase
            decel_time = 2.0      # seconds for deceleration phase
            max_velocity = 0.3    # maximum velocity in m/s
            accel = max_velocity / accel_time  # acceleration in m/s^2
            constant_time = (total_distance - 0.5 * accel_time * max_velocity - 0.5 * decel_time * max_velocity) / max_velocity
            total_time = accel_time + constant_time + decel_time

            if elapsed_time < accel_time:
                # Acceleration phase
                msg.linear.x = accel * elapsed_time
            elif elapsed_time < accel_time + constant_time:
                # Constant velocity phase
                msg.linear.x = max_velocity
            elif elapsed_time < total_time:
                # Deceleration phase
                decel_elapsed_time = elapsed_time - accel_time - constant_time
                msg.linear.x = max_velocity - accel * decel_elapsed_time
            else:
                msg.linear.x = 0.0
                self.robot_moving = False  # Stop moving the robot
                self.plot_odometry_data()  # Plot and save after stopping

        # Publish the velocity command
        self.publisher_.publish(msg)

    def odom_callback(self, msg):
        """Callback to process the odometry data from the /odom topic"""
        if self.robot_moving:
            current_time = time.time() - self.start_time

            # Extract x, y, z from the odometry message (Pose within Odometry message)
            position = msg.pose.pose.position
            self.x_data.append(position.x)
            self.y_data.append(position.y)
            self.z_data.append(position.z)
            self.time_data.append(current_time)

    def plot_odometry_data(self):
        """Plot the x, y, and z coordinates over time and save the plot"""
        plt.figure(figsize=(10, 6))

        # Plot x, y, and z coordinates over time
        plt.plot(self.time_data, self.x_data, label="x-coordinate", color='r')
        plt.plot(self.time_data, self.y_data, label="y-coordinate", color='g')
        plt.plot(self.time_data, self.z_data, label="z-coordinate", color='b')

        # Add labels and title
        plt.xlabel("Time (seconds)")
        plt.ylabel("Position (meters)")
        plt.title("TurtleBot3 Position (x, y, z) Over Time")

        # Add legend
        plt.legend()

        # Save the plot as an image
        plt.savefig('turtlebot_position_plot.png')
        self.get_logger().info("Odometry data plot saved as turtlebot_position_plot.png")

        # Show the plot (optional, you can comment it out if running headless)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    turtlebot_open_loop = TurtlebotOpenLoop()

    rclpy.spin(turtlebot_open_loop)
    turtlebot_open_loop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
