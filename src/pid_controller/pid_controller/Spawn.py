import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
import random
import math

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')

        
        self.spawn_client = self.create_client(Spawn, '/spawn')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for turtlesim_node to run...')
        
        self.cmd_vel_subscriber = self.create_subscription(Twist,'/turtle1/cmd_vel',self.cmd_vel_callback,10)
        self.spawn_turtle_at_random_location()

    def spawn_turtle_at_random_location(self):
        x = random.uniform(2.0, 8.0)  
        y = random.uniform(2.0, 8.0) 
        theta = random.uniform(0, 2 * math.pi)  

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = 'turtle1'  
        self.n = 1

    
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawned turtle{self.n} at ({x}, {y}) with orientation {theta}")
        else:
            self.get_logger().error("Failed to spawn turtle!")

    def cmd_vel_callback(self, msg):

       
        vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        cmd_vel = Twist()
        cmd_vel.linear.x = msg.linear.x
        cmd_vel.angular.z = msg.angular.z

        vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    move_turtle_node = MoveTurtle()
    rclpy.spin(move_turtle_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
