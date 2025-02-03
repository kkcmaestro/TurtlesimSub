import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from turtlesim.msg import Pose

class Reach_goal(Node):
    pose_x = 0 
    pose_y = 0
    theta = 0
    def __init__(self):
        super().__init__("no_pid_controller")
        self.velocity_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.01, self.reach_goal)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.set_goal()
    
    def set_goal(self):
        self.get_logger().info("Set the goal position for the turtle")
        self.x2 = float(input("X value: "))
        self.y2 = float(input("Y value: "))

    def pose_callback(self, pose_msg = Pose()):
        self.pose_x = pose_msg.x
        self.pose_y = pose_msg.y
        self.theta = pose_msg.theta

    def reach_goal(self):
        dist_x = self.x2 - self.pose_x
        dist_y = self.y2 - self.pose_y
        distance = math.sqrt(dist_x**2 + dist_y**2)
        distance_tolerance = 0.5

        velocity = Twist()
        if(distance > distance_tolerance):
            K_linear = 2
            velocity.linear.x = K_linear * distance

            angle = math.atan2(dist_y, dist_x)
            diff = angle - self.theta
            K_angular = 6 

            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < - math.pi:
                diff += 2*math.pi

            velocity.angular.z = K_angular*diff
        
        else:
            velocity.linear.x = 0.0
            velocity.angular.y = 0.0
            self.get_logger().info("Goal Reached!")
            self.timer.cancel()

        self.velocity_publisher_.publish(velocity)

def main(args=None):
    rclpy.init(args=args)
    reach_goal = Reach_goal()
    rclpy.spin(reach_goal)
    rclpy.shutdown()

if __name__ == "__main__":
    main()