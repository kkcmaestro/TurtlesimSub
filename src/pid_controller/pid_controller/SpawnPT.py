import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from std_msgs.msg import Float64MultiArray
import random
import time

class CircularTurtleController(Node):
    def __init__(self):
        super().__init__("circular_turtle_controller")

        # Publisher for turtle velocity commands
        self.vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Publishers for real and noisy pose
        self.real_pose_publisher = self.create_publisher(Float64MultiArray, "/rt_real_pose", 10)
        self.noisy_pose_publisher = self.create_publisher(Float64MultiArray, "/rt_noisy_pose", 10)

        # Subscriber for turtle pose
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        # Timer to update control loop (100 Hz)
        self.timer = self.create_timer(0.01, self.move_in_circle)

        # Timer to publish real & noisy pose every 5 seconds
        self.pose_timer = self.create_timer(5.0, self.publish_pose)

        # Timer to spawn second turtle after 10 seconds
        self.spawn_timer = self.create_timer(10.0, self.spawn_second_turtle)

        # Parameters
        self.linear_speed = 2.0   # Adjust this for speed
        self.angular_speed = 1.0  # Adjust this for turning rate
        self.noise_std_dev = 3.0  # Gaussian noise standard deviation

        # Pose variables
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.theta = 0.0
        self.second_turtle_spawned = False

    def pose_callback(self, pose_msg):
        """ Updates the turtle's current pose """
        self.pose_x = pose_msg.x
        self.pose_y = pose_msg.y
        self.theta = pose_msg.theta

    def move_in_circle(self):
        """ Moves the turtle in a circular path """
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_speed  # Move forward
        cmd_vel.angular.z = self.angular_speed  # Turn continuously
        self.vel_publisher.publish(cmd_vel)

    def spawn_second_turtle(self):
        """ Spawns the second turtle 10 seconds after the first one """
        if not self.second_turtle_spawned:
            # Call the /spawn service to spawn a second turtle
            spawn_client = self.create_client(Spawn, '/spawn')
            while not spawn_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again...')

            request = Spawn.Request()
            request.x = 5.0  # X position for second turtle
            request.y = 5.0  # Y position for second turtle
            request.theta = 0.0  # Orientation of second turtle
            request.name = 'turtle2'  # Name of the second turtle

            # Send the request to spawn the second turtle
            future = spawn_client.call_async(request)
            future.add_done_callback(self.spawn_callback)
    
    def spawn_callback(self, future):
        """ Callback once the second turtle is spawned """
        try:
            response = future.result()
            self.get_logger().info(f'Successfully spawned second turtle at ({response.x}, {response.y})')
            # Start controlling second turtle in circles
            self.control_second_turtle()
        except Exception as e:
            self.get_logger().error(f'Failed to spawn second turtle: {e}')

    def control_second_turtle(self):
        """ Controls the second turtle after spawning """
        # Create another publisher for the second turtle
        self.second_vel_publisher = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)
        # Start moving the second turtle in circles
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_speed  # Move forward
        cmd_vel.angular.z = self.angular_speed  # Turn continuously
        self.second_vel_publisher.publish(cmd_vel)

    def publish_pose(self):
        """ Publishes both real and noisy pose every 5 seconds """
        real_pose_msg = Float64MultiArray()
        noisy_pose_msg = Float64MultiArray()

        # Real pose (x, y, theta)
        real_pose_msg.data = [self.pose_x, self.pose_y, self.theta]
        self.real_pose_publisher.publish(real_pose_msg)

        # Noisy pose with Gaussian noise
        noisy_x = self.pose_x + random.gauss(0, self.noise_std_dev)
        noisy_y = self.pose_y + random.gauss(0, self.noise_std_dev)
        noisy_theta = self.theta + random.gauss(0, self.noise_std_dev)

        noisy_pose_msg.data = [noisy_x, noisy_y, noisy_theta]
        self.noisy_pose_publisher.publish(noisy_pose_msg)

        self.get_logger().info(f"Published Real Pose: {real_pose_msg.data}")
        self.get_logger().info(f"Published Noisy Pose: {noisy_pose_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    circular_turtle_controller = CircularTurtleController()
    rclpy.spin(circular_turtle_controller)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
