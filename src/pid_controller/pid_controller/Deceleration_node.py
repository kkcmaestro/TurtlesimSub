import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DecelerationNode(Node):
    def __init__(self):
        super().__init__('deceleration_node')
        
        self.subscription = self.create_subscription(Twist, '/turtle1/cmd_vel', self.cmd_vel_callback, 10)
        
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.decelerate)
        
        self.current_velocity = Twist()
        self.last_received_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        self.current_velocity = msg
        self.last_received_time = self.get_clock().now()

    def decelerate(self):
        time_since_last_cmd = (self.get_clock().now() - self.last_received_time).nanoseconds / 1e9

        #if it doesnt recieve new velocity command for 0.2 secs it starts to decelerate
        if time_since_last_cmd > 0.2:  
            if abs(self.current_velocity.linear.x) > 0.01:
                self.current_velocity.linear.x *= 0.9  
            else:
                self.current_velocity.linear.x = 0.0  

            if abs(self.current_velocity.angular.z) > 0.01:
                self.current_velocity.angular.z *= 0.9  
            else:
                self.current_velocity.angular.z = 0.0

            self.publisher.publish(self.current_velocity)  

def main():
    rclpy.init()
    node = DecelerationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
