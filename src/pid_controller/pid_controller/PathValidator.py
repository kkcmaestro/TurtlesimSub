import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
import time

class CircleEstimator(Node):
    def __init__(self):
        super().__init__("circle_estimator")
        
        self.rt_pose_subscriber = self.create_subscription(Float64MultiArray, "/rt_real_pose", self.rt_pose_callback, 10)
        self.timer = self.create_timer(5.0, self.calculate_circle)
        
        self.rt_positions = []
        self.velocity_publisher = self.create_publisher(Float64MultiArray, "/circle_info", 10)  
        self.last_time = time.time()
        
        self.previous_center = None

    def rt_pose_callback(self, msg):
        if len(self.rt_positions) >= 3:
            self.rt_positions.pop(0)
        self.rt_positions.append((msg.data[0], msg.data[1]))
    
    def calculate_circle(self):
        if len(self.rt_positions) < 3:
            return
        
        (x1, y1), (x2, y2), (x3, y3) = self.rt_positions
        
        if abs(x3 - x2) < 0.001 and abs(y3 - y2) < 0.001: 
            self.get_logger().info("RT is stationary. Publishing velocity as 0.")
            linear_velocity = 0.0  
            radius = 0.0
            h = k = 0.0
            circle_info_msg = Float64MultiArray()
            circle_info_msg.data = [linear_velocity, radius, h, k]
            self.velocity_publisher.publish(circle_info_msg)
            return
        
        A = np.array([
            [2 * (x2 - x1), 2 * (y2 - y1)],
            [2 * (x3 - x2), 2 * (y3 - y2)]
        ])
        
        B = np.array([
            x2**2 + y2**2 - x1**2 - y1**2,
            x3**2 + y3**2 - x2**2 - y2**2
        ])
        
        try:
            center = np.linalg.solve(A, B)
            h, k = center
            radius = math.sqrt((x1 - h)**2 + (y1 - k)**2)

            if self.previous_center is not None:
                prev_h, prev_k = self.previous_center
                if not math.isclose(h, prev_h, abs_tol=0.1) or not math.isclose(k, prev_k, abs_tol=0.1):
                    self.get_logger().info("Path is unpredictable. Not publishing circle info.")
                    self.previous_center = None  
                    return  
            
           
            self.previous_center = (h, k)

            
            angle = math.atan2(y2 - k, x2 - h) - math.atan2(y1 - k, x1 - h)
            if angle < 0:
                angle += 2 * math.pi

        
            arc_length = radius * angle

            
            dt = 5
            

            linear_velocity = arc_length / dt  

            if linear_velocity < 0.01:  
                linear_velocity = 0.0  
                self.get_logger().info("RT is stationary. Publishing velocity as shunya.")

    
            circle_info_msg = Float64MultiArray()
            circle_info_msg.data = [linear_velocity, radius, h, k]
            self.velocity_publisher.publish(circle_info_msg)

            self.get_logger().info(f'Path is circular. Center: ({h:.2f}, {k:.2f}), Radius: {radius:.2f}, Velocity: {linear_velocity:.2f} m/s')
        
        except np.linalg.LinAlgError:
            self.get_logger().error('Could not compute circle parameters')

def main(args=None):
    rclpy.init(args=args)
    node = CircleEstimator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
