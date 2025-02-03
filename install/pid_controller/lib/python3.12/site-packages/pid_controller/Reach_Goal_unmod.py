import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, atan2, pi

class PIDTurtleController(Node):
    def __init__(self):
        super().__init__("pid_turtle_controller")
        
        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.error_publisher = self.create_publisher(Float64, "/turtle1/linear_error", 10)
        self.angular_error_publisher = self.create_publisher(Float64, "/turtle1/angular_error", 10)
        
        # Subscriber to the turtle's current pose
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        # Timer to update control loop
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

        # Target position (User-defined)
        self.x_goal = float(input("Enter goal X: "))
        self.y_goal = float(input("Enter goal Y: "))

        # PID Controller Gains
        self.Kp_linear = 2.0
        self.Ki_linear = 0.0
        self.Kd_linear = 0.3

        self.Kp_angular = 5.0
        self.Ki_angular = 0.0
        self.Kd_angular = 0.5

        # Errors for PID calculations
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.integral_linear = 0.0
        self.integral_angular = 0.0

        # Turtle's pose
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.theta = 0.0

    def pose_callback(self, pose_msg):
        """ Updates the turtle's current pose """
        self.pose_x = pose_msg.x
        self.pose_y = pose_msg.y
        self.theta = pose_msg.theta

    def control_loop(self):
        """ PID control logic to move the turtle towards the goal """
        # Calculate distance and angle errors
        error_x = self.x_goal - self.pose_x
        error_y = self.y_goal - self.pose_y
        distance_error = sqrt(error_x**2 + error_y**2)
        angle_goal = atan2(error_y, error_x)
        angular_error = angle_goal - self.theta

        # Normalize angular error to [-pi, pi]
        if angular_error > pi:
            angular_error -= 2 * pi
        elif angular_error < -pi:
            angular_error += 2 * pi

        # Compute PID terms
        self.integral_linear += distance_error * 0.1  # Integral term
        derivative_linear = (distance_error - self.prev_linear_error) / 0.1  # Derivative term

        self.integral_angular += angular_error * 0.1
        derivative_angular = (angular_error - self.prev_angular_error) / 0.1

        # Compute PID outputs
        linear_velocity = (self.Kp_linear * distance_error +
                           self.Ki_linear * self.integral_linear +
                           self.Kd_linear * derivative_linear)

        angular_velocity = (self.Kp_angular * angular_error +
                            self.Ki_angular * self.integral_angular +
                            self.Kd_angular * derivative_angular)

        # Save current errors for next iteration
        self.prev_linear_error = distance_error
        self.prev_angular_error = angular_error

        # Create and publish velocity message
        cmd_vel = Twist()
        if distance_error > 0.1:  # If not close enough, move
            cmd_vel.linear.x = linear_velocity
            cmd_vel.angular.z = angular_velocity
        else:
            # Stop if the goal is reached
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.get_logger().info("Target reached!")
            self.timer.cancel()  # Stop the control loop

        self.error_publisher.publish(Float64(data=distance_error))
        self.angular_error_publisher.publish(Float64(data=angular_error))
        self.velocity_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDTurtleController()
    rclpy.spin(pid_controller)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
