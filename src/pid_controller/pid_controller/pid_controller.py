import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, atan2, pi

class PIDTurtleController(Node):
    def __init__(self):
        super().__init__("pid_turtle_controller")
        
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.error_publisher = self.create_publisher(Float64, "/turtle1/linear_error", 10)
        self.angular_error_publisher = self.create_publisher(Float64, "/turtle1/angular_error", 10)
        
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        self.timer = self.create_timer(0.01, self.control_loop) 

        # Takes user input
        self.x_goal = float(input("Enter goal X: "))
        self.y_goal = float(input("Enter goal Y: "))

        # PID gain values for linear motion
        self.Kp_linear = 2.0
        self.Ki_linear = 0.0
        self.Kd_linear = 0.3

        # PID gain values for angular motion
        self.Kp_angular = 5.0
        self.Ki_angular = 0.0
        self.Kd_angular = 0.5

        # initial error values
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.integral_linear = 0.0
        self.integral_angular = 0.0

        # Turtle's position
        self.X_position = 0.0
        self.Y_position = 0.0
        self.theta = 0.0

    def pose_callback(self, pose_msg):
        self.X_position = pose_msg.x
        self.Y_position = pose_msg.y
        self.theta = pose_msg.theta

    def control_loop(self):

        # Calculate distance and angle errors
        error_x = self.x_goal - self.X_position
        error_y = self.y_goal - self.Y_position
        distance_error = sqrt(error_x**2 + error_y**2)
        angle_goal = atan2(error_y, error_x)
        angular_error = angle_goal - self.theta


        if angular_error > pi:
            angular_error -= 2 * pi
        elif angular_error < -pi:
            angular_error += 2 * pi

        # calculating pid terms
        self.integral_linear += distance_error * 0.1 #took 0.1 as a abitrary value
        derivative_linear = (distance_error - self.prev_linear_error) / 0.1  

        self.integral_angular += angular_error * 0.1
        derivative_angular = (angular_error - self.prev_angular_error) / 0.1

        # outpuit generation
        linear_velocity = (self.Kp_linear * distance_error +
                           self.Ki_linear * self.integral_linear +
                           self.Kd_linear * derivative_linear)

        angular_velocity = (self.Kp_angular * angular_error +
                            self.Ki_angular * self.integral_angular +
                            self.Kd_angular * derivative_angular)

        #saving values for next iteration
        self.prev_linear_error = distance_error
        self.prev_angular_error = angular_error

        
        cmd_vel = Twist()
        if distance_error > 0.1:  
            cmd_vel.linear.x = linear_velocity
            cmd_vel.angular.z = angular_velocity
        else:
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
