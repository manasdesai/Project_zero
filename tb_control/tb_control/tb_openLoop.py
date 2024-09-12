import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtlebotOpenLoop(Node):
    def __init__(self):
        super().__init__('tb_openLoop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.scenario = 2  # Change to 2 for the second scenario
        self.start_time = time.time()

    def timer_callback(self):
        elapsed_time = time.time() - self.start_time
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
                # Stop
                msg.linear.x = 0.0

        # Publish the velocity command
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_open_loop = TurtlebotOpenLoop()

    rclpy.spin(turtlebot_open_loop)
    turtlebot_open_loop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()