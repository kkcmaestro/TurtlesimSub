import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from std_msgs.msg import Float64MultiArray
import math
import random

class PoliceTurtle(Node):
    def __init__(self):
        super().__init__("police_turtle")

        self.vel_publisher = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

        self.rt_pose_subscriber = self.create_subscription(Float64MultiArray, "/rt_real_pose", self.rt_pose_callback, 10)
        self.pt_pose_subscriber = self.create_subscription(Pose, "/turtle2/pose", self.pt_pose_callback, 10)

        self.timer = self.create_timer(0.1, self.chase_robber)

        # calling the function to spawn police turtle
        self.Police_turtle_spawn()

        # Parameters
        self.max_speed = 2.0  
        self.max_acceleration = 0.5  
        self.max_deceleration = 1.0  

        # linear PID gains
        self.kp_linear = 1.2
        self.ki_linear = 0.0
        self.kd_linear = 0.2

        #angular pid gains
        self.kp_angular = 4.0
        self.ki_angular = 0.0
        self.kd_angular = 0.3

        
        self.rt_pose = None  # RT's position
        self.pt_pose = None  # PT's own position

        #error terms
        self.prev_linear_velocity = 0.0
        self.integral_error_linear = 0.0
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0

        self.dt = 0.1  

        #assumed values
        self.RT_velocity = 2.0  
        self.RT_Radius = 5.0  
        self.RT_path_center = (5.0, 5.0)  
        self.target_reached = False  

    def Police_turtle_spawn(self):
        self.spawn_client = self.create_client(Spawn, "/spawn")
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for turtlesim_node to start...')

        request = Spawn.Request()

        #randomizing the spawn point
        request.x = random.uniform(2.0, 8.0)  
        request.y = random.uniform(2.0, 8.0)
        request.theta = random.uniform(0, 2 * math.pi)
        request.name = "turtle2"

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Police Turtle (PT) spawned at ({request.x}, {request.y})")
        else:
            self.get_logger().error("Failed to spawn Police Turtle!")
    
    def rt_pose_callback(self, msg):
     
        #this block stops futher motion if the robber turtle is caught
        if self.target_reached:
            return  
        
        RT_x_pos, RT_y_pos = msg.data[0], msg.data[1]
        Passed_time = 5.0 

        theta_current = math.atan2(RT_y_pos - self.RT_path_center[1], RT_x_pos - self.RT_path_center[0])
        theta_predicted = theta_current + (self.RT_velocity / self.RT_Radius) * Passed_time
        predicted_x = (self.RT_path_center[0] + self.RT_Radius * math.cos(1*theta_predicted))
        predicted_y = (self.RT_path_center[1] + self.RT_Radius * math.sin(1*theta_predicted))
        self.rt_pose = (predicted_x, predicted_y)

        self.get_logger().info(f"Predicted RT Position: {self.rt_pose}")

    def pt_pose_callback(self, msg):
        self.pt_pose = (msg.x, msg.y, msg.theta)

    def chase_robber(self):
        if self.rt_pose is None or self.pt_pose is None:
            return  

        # current position of police turle
        police_x, police_y, police_theta_angle = self.pt_pose

        # error calculation
        error_x = self.rt_pose[0] - police_x
        error_y = self.rt_pose[1] - police_y
        error_dist = math.sqrt(error_x**2 + error_y**2)

        #run this when the turtle is caught
        if error_dist <= 0.5:
            self.get_logger().info("RT has been caught! ")
            self.vel_publisher.publish(Twist()) 
            return

        # Calculating Pid values
        self.integral_error_linear += error_dist * self.dt
        derivative_error_linear = (error_dist - self.prev_error_linear) / self.dt
        self.prev_error_linear = error_dist

        raw_linear_velocity = (
            self.kp_linear * error_dist +
            self.ki_linear * self.integral_error_linear +
            self.kd_linear * derivative_error_linear
        )
        raw_linear_velocity = min(self.max_speed, raw_linear_velocity)  # Limit max speed

        # Applying limits to both velocity and accelration
        velocity_change = raw_linear_velocity - self.prev_linear_velocity
        if velocity_change > self.max_acceleration * self.dt:
            linear_velocity = self.prev_linear_velocity + self.max_acceleration * self.dt
        elif velocity_change < -self.max_deceleration * self.dt:
            linear_velocity = self.prev_linear_velocity - self.max_deceleration * self.dt
        else:
            linear_velocity = raw_linear_velocity

        desired_angle = math.atan2(error_y, error_x)
        angle_error = desired_angle - police_theta_angle

        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        derivative_error_angular = (angle_error - self.prev_error_angular) / self.dt
        self.prev_error_angular = angle_error

        angular_velocity = (
            self.kp_angular * angle_error +
            self.kd_angular * derivative_error_angular
        )

        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        self.vel_publisher.publish(cmd_vel)

        self.prev_linear_velocity = linear_velocity

def main(args=None):
    rclpy.init(args=args)
    police_turtle = PoliceTurtle()
    rclpy.spin(police_turtle)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
