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

        # Publisher for velocity commands
        self.vel_publisher = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

        # Subscribers for RT's position & PT's own pose
        self.rt_pose_subscriber = self.create_subscription(Float64MultiArray, "/rt_real_pose", self.rt_pose_callback, 10)
        self.pt_pose_subscriber = self.create_subscription(Pose, "/turtle2/pose", self.pt_pose_callback, 10)

        # Timer to control movement
        self.timer = self.create_timer(0.1, self.chase_robber)

        # Spawn PT at a random location
        self.spawn_police()

        # Parameters
        self.max_speed = 1.0  # PT moves faster than RT
        self.max_acceleration = 0.5  # Acceleration limit
        self.max_deceleration = 1.0  # Deceleration limit

        # Separate PID Gains for Linear & Angular Motion
        self.kp_linear = 1.2
        self.ki_linear = 0.0
        self.kd_linear = 0.2

        self.kp_angular = 4.0
        self.ki_angular = 0.0
        self.kd_angular = 0.3

        # Internal state
        self.rt_pose = None  # RT's position
        self.pt_pose = None  # PT's own position
        self.prev_linear_velocity = 0.0
        self.integral_error_linear = 0.0
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.dt = 0.1  # Control loop timing
        self.rt_speed = 2.0  # Assumed constant speed of RT
        self.rt_radius = 5.0  # Assumed constant radius of RT's motion
        self.rt_center = (5.0, 5.0)  # Assumed center of RT's circular motion
        self.target_reached = False  # Flag to stop tracking once RT is caught

    def spawn_police(self):
        """ Spawns PT at a random location """
        self.spawn_client = self.create_client(Spawn, "/spawn")
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        request = Spawn.Request()
        request.x = random.uniform(2.0, 8.0)  # Avoid spawning near RT
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
        """ Updates RT's real pose and predicts its future position """
        if self.target_reached:
            return  # Ignore new pose updates if RT is already caught
        
        rt_x, rt_y = msg.data[0], msg.data[1]
        elapsed_time = 5.0  # RT updates every 5 seconds
        theta_current = math.atan2(rt_y - self.rt_center[1], rt_x - self.rt_center[0])
        theta_predicted = theta_current + (self.rt_speed / self.rt_radius) * elapsed_time
        predicted_x = self.rt_center[0] + self.rt_radius * math.cos(2*theta_predicted)
        predicted_y = self.rt_center[1] + self.rt_radius * math.sin(2*theta_predicted)
        self.rt_pose = (predicted_x, predicted_y)
        self.get_logger().info(f"Predicted RT Position: {self.rt_pose}")

    def pt_pose_callback(self, msg):
        """ Updates PT's real-time position """
        self.pt_pose = (msg.x, msg.y, msg.theta)

    def chase_robber(self):
        """ Uses PID to chase RT with acceleration limits """
        if self.rt_pose is None or self.pt_pose is None or self.target_reached:
            return  # Stop tracking once RT is caught

        # PT's current position
        pt_x, pt_y, pt_theta = self.pt_pose

        # Compute error (distance to RT)
        error_x = self.rt_pose[0] - pt_x
        error_y = self.rt_pose[1] - pt_y
        error_dist = math.sqrt(error_x**2 + error_y**2)

        # If PT is close enough, stop moving and print message
        if error_dist <= 0.5:
            self.get_logger().info("RT has been caught! ✅")
 
            if self.target_reached:  # ✅ Prevents further movement after catching RT
                self.vel_publisher.publish(Twist())  # Ensure PT fully stops
                return

        # --- Linear Velocity Control (PID) ---
        self.integral_error_linear += error_dist * self.dt
        derivative_error_linear = (error_dist - self.prev_error_linear) / self.dt
        self.prev_error_linear = error_dist

        raw_linear_velocity = (
            self.kp_linear * error_dist +
            self.ki_linear * self.integral_error_linear +
            self.kd_linear * derivative_error_linear
        )
        raw_linear_velocity = min(self.max_speed, raw_linear_velocity)  # Limit max speed

        # Enforce acceleration/deceleration constraints
        velocity_change = raw_linear_velocity - self.prev_linear_velocity
        if velocity_change > self.max_acceleration * self.dt:
            linear_velocity = self.prev_linear_velocity + self.max_acceleration * self.dt
        elif velocity_change < -self.max_deceleration * self.dt:
            linear_velocity = self.prev_linear_velocity - self.max_deceleration * self.dt
        else:
            linear_velocity = raw_linear_velocity

        # --- Angular Velocity Control (PID) ---
        desired_angle = math.atan2(error_y, error_x)
        angle_error = desired_angle - pt_theta

        # Normalize angle to [-pi, pi]
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

        # Publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        self.vel_publisher.publish(cmd_vel)

        # Update previous velocity
        self.prev_linear_velocity = linear_velocity

def main(args=None):
    rclpy.init(args=args)
    police_turtle = PoliceTurtle()
    rclpy.spin(police_turtle)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
