import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from std_msgs.msg import Float64MultiArray
import math
import random

# the kalman filter class to filter out noisy value
class KalmanFilter:
    def __init__(self):
        #values are taken as per usual norm
        self.x = np.array([[0.0], [0.0]])  
        self.P = np.eye(2) * 1000  
        self.F = np.eye(2)  
        self.H = np.eye(2)  
        self.R = np.eye(2) * 9.0  
        self.Q = np.eye(2) * 0.1  

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x.flatten()

    def update(self, measurement):
        z = np.array([[measurement[0]], [measurement[1]]])
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(K, np.dot(self.H, self.P))
        return self.x.flatten()

class PoliceTurtle(Node):
    def __init__(self):
        super().__init__("police_turtle")

       
        self.vel_publisher = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

      
        self.rt_pose_subscriber = self.create_subscription(Float64MultiArray, "/rt_real_pose", self.rt_pose_callback, 10)
        self.pt_pose_subscriber = self.create_subscription(Pose, "/turtle2/pose", self.pt_pose_callback, 10)

      
        self.timer = self.create_timer(0.1, self.chase_robber)

     
        self.Police_turtle_spawn()

        
        self.max_speed = 2.0  
        self.max_acceleration = 0.5  
        self.max_deceleration = 1.0  

        self.kp_linear = 1.2
        self.ki_linear = 0.0
        self.kd_linear = 0.2

        self.kp_angular = 4.0
        self.ki_angular = 0.0
        self.kd_angular = 0.3

       
        self.rt_pose = None  
        self.pt_pose = None  
        self.target_reached = False 
        self.kalman_filter = KalmanFilter() 

    def Police_turtle_spawn(self):
        
        self.spawn_client = self.create_client(Spawn, "/spawn")
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        request = Spawn.Request()

        request.x = random.uniform(2.0, 8.0)  
        request.y = random.uniform(2.0, 8.0)

        request.theta = random.uniform(0, 2 * math.pi)
        request.name = "turtle2"

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Police Turtle spawned at ({request.x}, {request.y})")
        else:
            self.get_logger().error("Failed to spawn Police Turtle!")
    
    def rt_pose_callback(self, msg):
        
        if self.rt_pose is None:  
            noisy_x, noisy_y = msg.data[0], msg.data[1]
            rt_x, rt_y = self.kalman_filter.update([noisy_x, noisy_y])

            elapsed_time = 5.0  
            theta_current = math.atan2(rt_y - 5.0, rt_x - 5.0)  
            theta_predicted = theta_current + (2.0 / 5.0) * elapsed_time
            predicted_x = 5.0 + 5.0 * math.cos(1*theta_predicted)
            predicted_y = 5.0 + 5.0 * math.sin(1*theta_predicted)

            self.rt_pose = (predicted_x, predicted_y)

            self.get_logger().info(f"Predicted RT Position: {self.rt_pose}")

    def pt_pose_callback(self, msg):
        self.pt_pose = (msg.x, msg.y, msg.theta)

    def chase_robber(self):
  
        if self.rt_pose is None or self.pt_pose is None or self.target_reached:
            return  

        pt_x, pt_y, pt_theta = self.pt_pose
        error_x = self.rt_pose[0] - pt_x
        error_y = self.rt_pose[1] - pt_y
        error_dist = math.sqrt(error_x**2 + error_y**2)

        if error_dist <= 1.5:
            self.get_logger().info("RT has been caught!")
            self.vel_publisher.publish(Twist())  
            self.target_reached = True
            return

      
        desired_angle = math.atan2(error_y, error_x)
        angle_error = desired_angle - pt_theta
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        linear_velocity = min(self.max_speed, self.kp_linear * error_dist)
        angular_velocity = self.kp_angular * angle_error

     
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        self.vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    police_turtle = PoliceTurtle()
    rclpy.spin(police_turtle)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
