import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float64MultiArray, Bool
import random

class CircularTurtle(Node):
    def __init__(self):
        super().__init__("Robber_turtle_circular_motion")

        self.vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.real_pose_publisher = self.create_publisher(Float64MultiArray, "/rt_real_pose", 10)
        self.noisy_pose_publisher = self.create_publisher(Float64MultiArray, "/rt_noisy_pose", 10)


        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.catch_subscriber = self.create_subscription(Bool, "/rt_caught", self.catch_callback, 10)

     
        self.timer = self.create_timer(0.01, self.Circular_motion)

      
        self.pose_timer = self.create_timer(5.0, self.publish_pose)

        self.Radius = 3.5
        self.linear_velocity = 5.0  
        self.angular_velocity = self.linear_velocity / self.Radius 

        self.noise_std_dev = 3.0  

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.theta = 0.0

        self.caught = False  

    def pose_callback(self, pose_msg):
        self.pose_x = pose_msg.x
        self.pose_y = pose_msg.y
        self.theta = pose_msg.theta

    def catch_callback(self, msg):
        if msg.data:  
            self.caught = True
            self.get_logger().info("RT has been caught! Stopping movement.")
            self.stop_movement()

    def Circular_motion(self):
        if not self.caught:
            cmd_vel = Twist()
            cmd_vel.linear.x = self.linear_velocity 
            cmd_vel.angular.z = self.angular_velocity  
            self.vel_publisher.publish(cmd_vel)

    def stop_movement(self):
        stop_msg = Twist()
        self.vel_publisher.publish(stop_msg)

    def publish_pose(self):
        if self.caught:
            return  

        real_pose_msg = Float64MultiArray()
        noisy_pose_msg = Float64MultiArray()

        real_pose_msg.data = [self.pose_x, self.pose_y, self.theta]
        self.real_pose_publisher.publish(real_pose_msg)

        noisy_x = self.pose_x + random.gauss(0, self.noise_std_dev)
        noisy_y = self.pose_y + random.gauss(0, self.noise_std_dev)
        noisy_theta = self.theta + random.gauss(0, self.noise_std_dev)

        noisy_pose_msg.data = [noisy_x, noisy_y, noisy_theta]
        self.noisy_pose_publisher.publish(noisy_pose_msg)

        self.get_logger().info(f"Published Real Pose: {real_pose_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    circular_turtle = CircularTurtle()
    rclpy.spin(circular_turtle)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
