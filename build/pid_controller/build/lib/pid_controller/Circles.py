import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float64MultiArray
import random
import time

class CircularTurtle(Node):
    def __init__(self):
        super().__init__("Robber_turtle_circular_motiom")

        #initiation publishers
        self.vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.real_pose_publisher = self.create_publisher(Float64MultiArray, "/rt_real_pose", 10)
        self.noisy_pose_publisher = self.create_publisher(Float64MultiArray, "/rt_noisy_pose", 10)

        #initiating subscriber
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        #regular timer running every 0.01 seconds
        self.timer = self.create_timer(0.01, self.Circular_motion)

        #timer to publish every 5 seconds
        self.pose_timer = self.create_timer(5.0, self.publish_pose)

        # Parameters
        self.Radius = 5.0
        self.linear_velocity = 2.0   
        self.angular_velocity = self.linear_velocity / self.Radius 

        # Gaussian noise standard deviation
        self.noise_std_dev = 3.0  

        # init pose at 0.0 for every param
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.theta = 0.0

    def pose_callback(self, pose_msg):
        self.pose_x = pose_msg.x
        self.pose_y = pose_msg.y
        self.theta = pose_msg.theta

    def Circular_motion(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_velocity 
        cmd_vel.angular.z = self.angular_velocity  
        #publishing cmd_vel
        self.vel_publisher.publish(cmd_vel)

    def publish_pose(self):
        real_pose_msg = Float64MultiArray()
        noisy_pose_msg = Float64MultiArray()

        # publisher for real pose
        real_pose_msg.data = [self.pose_x, self.pose_y, self.theta]
        self.real_pose_publisher.publish(real_pose_msg)

        # publisher for noisy pose
        noisy_x = self.pose_x + random.gauss(0, self.noise_std_dev)
        noisy_y = self.pose_y + random.gauss(0, self.noise_std_dev)
        noisy_theta = self.theta + random.gauss(0, self.noise_std_dev)

        noisy_pose_msg.data = [noisy_x, noisy_y, noisy_theta]
        self.noisy_pose_publisher.publish(noisy_pose_msg)

        #self.get_logger().info(f"Published Real Pose: {real_pose_msg.data}")
        self.get_logger().info(f"Published Noisy Pose: {noisy_pose_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    circular_turtle = CircularTurtle()
    rclpy.spin(circular_turtle)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
