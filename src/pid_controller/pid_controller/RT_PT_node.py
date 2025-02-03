import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
import random
import time

class RobberTurtle(Node):
    """ RT moves in a circle (radius = 5) """
    def __init__(self):
        super().__init__('robber_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_publisher = self.create_publisher(Pose, 'rt_real_pose', 10)
        self.timer = self.create_timer(0.1, self.move_in_circle)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Get start time

    def move_in_circle(self):
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        vel_msg = Twist()
        Radius = 5.0
        vel_msg.linear.x = 2.0  # Constant speed
        vel_msg.angular.z = vel_msg.linear.x / Radius  # Omega = v/r
        self.publisher_.publish(vel_msg)

        # Simulating RT position
        rt_pose = Pose()
        rt_pose.x = 5 + 5 * math.cos(elapsed_time * vel_msg.angular.z)
        rt_pose.y = 5 + 5 * math.sin(elapsed_time * vel_msg.angular.z)
        self.pose_publisher.publish(rt_pose)


class PoliceTurtle(Node):
    """ PT chases RT using PID control & acceleration limits """
    def __init__(self):
        super().__init__('police_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'rt_real_pose', self.update_target, 10)

        self.prev_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.target_x, self.target_y = None, None

        # Spawn PT randomly
        self.pt_x, self.pt_y = random.uniform(1, 9), random.uniform(1, 9)
        self.pt_theta = 0.0
        self.prev_velocity = 0.0

        # PID Gains
        self.kp = 1.5
        self.ki = 0.0
        self.kd = 0.5

        self.prev_error = 0.0
        self.integral = 0.0

        # Limits
        self.max_acceleration = 1.0
        self.max_deceleration = 1.0

        self.timer = self.create_timer(0.1, self.chase_robber)

    def update_target(self, msg):
        """ Gets RT's position every 5s """
        self.target_x, self.target_y = msg.x, msg.y

    def pid_control(self, error, dt):
        """ Computes PID velocity command """
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        control = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return control

    def enforce_acceleration_limits(self, velocity, dt):
        """ Ensures PT obeys acceleration constraints """
        velocity_change = velocity - self.prev_velocity
        max_velocity_change = self.max_acceleration * dt
        min_velocity_change = -self.max_deceleration * dt

        if velocity_change > max_velocity_change:
            velocity = self.prev_velocity + max_velocity_change
        elif velocity_change < min_velocity_change:
            velocity = self.prev_velocity + min_velocity_change

        self.prev_velocity = velocity
        return velocity

    def chase_robber(self):
        """ Controls PT to chase RT smoothly using PID """
        if self.target_x is None or self.target_y is None:
            return  # No update yet

        dt = self.get_clock().now().seconds_nanoseconds()[0] - self.prev_time
        self.prev_time = self.get_clock().now().seconds_nanoseconds()[0]
        if dt <= 0:  # Avoid division by zero
            return

        # Compute distance to RT
        error_x = self.target_x - self.pt_x
        error_y = self.target_y - self.pt_y
        distance = math.sqrt(error_x**2 + error_y**2)

        if distance <= 0.5:
            self.get_logger().info("Chase complete! RT caught.")
            return

        # PID control for linear velocity
        linear_velocity = self.pid_control(distance, dt)
        linear_velocity = self.enforce_acceleration_limits(linear_velocity, dt)

        # Compute angular velocity to turn towards RT
        target_theta = math.atan2(error_y, error_x)
        angle_error = target_theta - self.pt_theta
        angular_velocity = self.kp * angle_error

        # Update PT position for simulation
        self.pt_x += linear_velocity * math.cos(self.pt_theta) * dt
        self.pt_y += linear_velocity * math.sin(self.pt_theta) * dt
        self.pt_theta += angular_velocity * dt

        # Publish velocity
        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity
        self.publisher_.publish(vel_msg)

def main():
    rclpy.init()
    robber = RobberTurtle()
    police = PoliceTurtle()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robber)
    executor.add_node(police)
    executor.spin()
    robber.destroy_node()
    police.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
