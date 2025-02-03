import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, atan2, pi

class ZigzagTurtle(Node):
    def __init__(self):
        super().__init__("zigzag_turtle_controller")
        

        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.error_publisher = self.create_publisher(Float64, "/turtle1/linear_error", 10)
        self.angular_error_publisher = self.create_publisher(Float64, "/turtle1/angular_error", 10)
        self.acceleration_publisher = self.create_publisher(Float64, "/turtle1/acceleration", 10)
    

        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        self.timer = self.create_timer(0.01, self.control_loop)  

        # Define grid limits
        self.grid_x_min, self.grid_x_max = 1.0, 9.0
        self.grid_y_min, self.grid_y_max = 1.0, 9.0
        #the space between them
        self.y_step = 1.0  

        self.path = self.generate_zigzag_path()
        self.initial_goal = 0  
        self.set_new_goal()

        self.Kp_linear = 2.0
        self.Ki_linear = 0.0
        self.Kd_linear = 0.3

        self.Kp_angular = 5.0
        self.Ki_angular = 0.0
        self.Kd_angular = 0.5

       #limits
        self.max_acceleration = 1.0  
        self.max_deceleration = 1.5  

        
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.integral_linear = 0.0
        self.integral_angular = 0.0

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.theta = 0.0

        self.prev_velocity = 0.0
        self.turning = False  

    def generate_zigzag_path(self):
    
        points = []
        y = self.grid_y_min
        #would use 1 for postion and -1 for negavity axis motion
        direction = 1  

        while y <= self.grid_y_max:
            if direction == 1:
                points.append((self.grid_x_max, y))  
            else:
                points.append((self.grid_x_min, y)) 
            
            y += self.y_step
            if y <= self.grid_y_max:
                points.append((points[-1][0], y))  
            
            direction *= -1  
        return points

    def set_new_goal(self):

        if self.initial_goal < len(self.path):
            self.x_goal, self.y_goal = self.path[self.initial_goal]
            self.get_logger().info(f"New goal set: ({self.x_goal}, {self.y_goal})")
            self.initial_goal += 1
            self.turning = True  
        else:
            self.get_logger().info("Zigzag traversal complete!")
            self.timer.cancel()  

    def pose_callback(self, pose_msg):
 
        self.pose_x = pose_msg.x
        self.pose_y = pose_msg.y
        self.theta = pose_msg.theta

    def control_loop(self):
   
        error_x = self.x_goal - self.pose_x
        error_y = self.y_goal - self.pose_y
        distance_error = sqrt(error_x**2 + error_y**2)
        angle_goal = atan2(error_y, error_x)
        angular_error = angle_goal - self.theta

  
        if angular_error > pi:
            angular_error -= 2 * pi
        elif angular_error < -pi:
            angular_error += 2 * pi


        if self.turning and abs(angular_error) > 0.01:
            cmd_vel = Twist()
            cmd_vel.angular.z = self.Kp_angular * angular_error
            self.velocity_publisher.publish(cmd_vel)
            return

  
        self.integral_linear += distance_error * 0.01
        derivative_linear = (distance_error - self.prev_linear_error) / 0.01

        self.integral_angular += angular_error * 0.01
        derivative_angular = (angular_error - self.prev_angular_error) / 0.01

      
        raw_linear_velocity = (self.Kp_linear * distance_error +
                               self.Ki_linear * self.integral_linear +
                               self.Kd_linear * derivative_linear)

        raw_angular_velocity = (self.Kp_angular * angular_error +
                                self.Ki_angular * self.integral_angular +
                                self.Kd_angular * derivative_angular)

   
        velocity_change = raw_linear_velocity - self.prev_velocity
        if velocity_change > self.max_acceleration * 0.01:
            linear_velocity = self.prev_velocity + self.max_acceleration * 0.01
        elif velocity_change < -self.max_deceleration * 0.1:
            linear_velocity = self.prev_velocity - self.max_deceleration * 0.01
        else:
            linear_velocity = raw_linear_velocity
        
        acceleration = (linear_velocity - self.prev_velocity) /0.01

    
        self.prev_linear_error = distance_error
        self.prev_angular_error = angular_error
        self.prev_velocity = linear_velocity

    
        if distance_error < 0.1:
            self.get_logger().info("Target reached! Moving to next goal...")
            self.set_new_goal()

     
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = raw_angular_velocity
        self.velocity_publisher.publish(cmd_vel)

        # publish errors
        self.error_publisher.publish(Float64(data=distance_error))
        self.angular_error_publisher.publish(Float64(data=angular_error))
        self.acceleration_publisher.publish(Float64(data=acceleration))

def main(args=None):
    rclpy.init(args=args)
    zigzag_controller = ZigzagTurtle()
    rclpy.spin(zigzag_controller)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
