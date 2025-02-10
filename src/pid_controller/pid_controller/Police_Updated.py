import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from std_msgs.msg import Float64MultiArray, Bool
import math
import random
import time

class PoliceTurtle(Node):

    def __init__(self):
        super().__init__("police_turtle")

        # Publishers
        self.vel_publisher = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)
        self.reached_goal_publisher = self.create_publisher(Bool, "/rt_caught", 10)  # Publisher for goal status

        # Subscribers
        self.rt_pose_subscriber = self.create_subscription(Float64MultiArray, "/rt_real_pose", self.rt_pose_callback, 10)
        self.pt_pose_subscriber = self.create_subscription(Pose, "/turtle2/pose", self.pt_pose_callback, 10)
        self.circle_info_subscriber = self.create_subscription(Float64MultiArray, "/circle_info", self.circle_info_callback, 10)

      
        self.timer = self.create_timer(0.01, self.chase_robber)

     
        self.Police_turtle_spawn()

       
        self.max_speed = 1.0  
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

        self.prev_linear_velocity = 0.0
        self.integral_error_linear = 0.0
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0

        self.dt = 0.01  

        self.RT_velocity = None  
        self.RT_Radius = None  
        self.RT_path_center = None 
        self.circle_info = None 

        self.vel_ratio = None
        self.predicted_goal = None  
        self.predicted_time = None  
        self.waiting = False  
        self.wait_timer = None  

        self.prediction_start_time = None 

    def Police_turtle_spawn(self):
        self.spawn_client = self.create_client(Spawn, "/spawn")
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for turtlesim_node to start...')

        request = Spawn.Request()
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

    def circle_info_callback(self, msg):
        if self.predicted_goal is None:  
            self.circle_info = msg.data
            self.RT_velocity, self.RT_Radius, self.RT_path_center_x, self.RT_path_center_y = self.circle_info
            self.RT_path_center = (self.RT_path_center_x, self.RT_path_center_y)
            self.vel_ratio = self.RT_velocity / self.max_speed

            self.predicted_goal = self.rt_pose
            self.update_predicted_time()  
            self.waiting = False
            self.wait_timer = 0
            self.prediction_start_time = time.time() 
            self.get_logger().info(f"Predicted Goal Set at {self.predicted_goal}, Time to Wait: {self.predicted_time}s")

    def update_predicted_time(self):
        """Recalculate the predicted time to complete one full circle dynamically."""
        if self.RT_velocity > 0 and self.RT_Radius > 0:
            self.predicted_time = (2 * math.pi * self.RT_Radius) / self.RT_velocity
        else:
            self.predicted_time = None 

    def rt_pose_callback(self, msg):
        self.rt_pose = (msg.data[0], msg.data[1])

    def pt_pose_callback(self, msg):
        self.pt_pose = (msg.x, msg.y, msg.theta)

    def chase_robber(self):
        if self.rt_pose is None or self.pt_pose is None or self.predicted_goal is None:
            return  

        police_x, police_y, police_theta_angle = self.pt_pose
        error_x = self.predicted_goal[0] - police_x
        error_y = self.predicted_goal[1] - police_y
        error_dist = math.sqrt(error_x**2 + error_y**2)

       
        self.update_predicted_time()

       
        if self.predicted_time is not None:
            elapsed_time = time.time() - self.prediction_start_time
            if elapsed_time >= self.predicted_time:
                self.get_logger().info(f"Prediction time elapsed ({elapsed_time}s), checking if PT has caught RT.")

                
                if error_dist <= 0.5:
                    self.get_logger().info("PT reached goal, stopping movement.")
                    self.vel_publisher.publish(Twist())  

                    
                    goal_reached_msg = Bool()
                    goal_reached_msg.data = True
                    self.reached_goal_publisher.publish(goal_reached_msg)
                    return

        desired_angle = math.atan2(error_y, error_x)
        angle_error = desired_angle - police_theta_angle
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        cmd_vel = Twist()
        cmd_vel.linear.x = min(self.max_speed, error_dist * self.kp_linear)
        cmd_vel.angular.z = self.kp_angular * angle_error
        self.vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    police_turtle = PoliceTurtle()
    rclpy.spin(police_turtle)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
