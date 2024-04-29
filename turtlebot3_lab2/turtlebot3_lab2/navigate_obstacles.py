import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan as Scan
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, TeleportAbsolute
from turtlesim.srv import SetPen
from turtlesim.srv import Kill
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float64MultiArray
import math
import time
import random
import math


class NavigateObstacles(Node):
    def __init__(self):
        super().__init__('turtle_follower')

        self.display = True
        self.declare_parameter('distance_threshold', 0.5)
        self.declare_parameter('clockwise_turn', True)
        # Initialization of publisher
        self.cmdvel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialization of main timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Initialization of scan subscription
        self.scan_subscriber = self.create_subscription(Scan, '/scan', self.print_scan, 10)
        
        self.right_distance = 999
        self.left_distance = 999
        self.left_forward_distance = 999
        self.left_backward_distance = 999
        self.distance = 999
        
        self.threshold = self.get_parameter('distance_threshold').value
        self.turn_cw = self.get_parameter('clockwise_turn').value
        
        self.state = "searching_for_wall"
        
        self.counter = 0
        self.distance_array = []
        self.data = []

    def print_scan(self, data):
        self.data = data

        forward_index = 360

        right_index = 540
        
        lf_index = 240
        
        lb_index = 120

        left_index = 180

        back_r = 690
        back_l = 30

        new_distance = data.ranges[back_r]
        self.back_r_distance = new_distance

        new_distance = data.ranges[back_l]
        self.back_l_distance = new_distance



        new_distance = data.ranges[forward_index]
        self.distance = new_distance
        
        new_distance = data.ranges[right_index]
        self.right_distance = new_distance
        
        new_distance = data.ranges[left_index]
        self.left_distance = new_distance
            
        new_distance = data.ranges[lf_index]
        self.left_forward_distance = new_distance
        
        new_distance = data.ranges[lb_index]
        self.left_backward_distance = new_distance

    def timer_callback(self):
        message = Twist()

        if self.state == 'searching_for_wall':
            if self.distance < self.threshold:
                self.state = 'rotating_at_first_wall' if self.counter == 0 else 'rotating_at_corner'
                self.counter += 1
                message.linear.x = 0.0
                message.angular.z = 0.0
            else:
                if self.left_backward_distance > (2.0 / math.sqrt(3)) * (self.left_distance) or self.left_distance < self.threshold:
                    message.angular.z = -0.05
                elif self.left_backward_distance < (2.0 / math.sqrt(3)) * (self.left_distance):
                    message.angular.z = 0.05
                else:
                    message.angular.z = 0.0
                message.linear.x = 0.5
        
        elif self.state == 'rotating_at_first_wall':
            if self.left_backward_distance <= self.left_forward_distance:
                self.state = 'searching_for_wall'
                message.angular.z = 0.0
            else:
                message.angular.z = -0.1
                message.linear.x = -0.018
        
        elif self.state == 'rotating_at_corner':
            self.get_logger().info("dist: %s" % str(self.distance))
            self.get_logger().info("rb: %s" % str(self.left_backward_distance))
            self.get_logger().info("rf: %s" % str(self.left_forward_distance))
            if self.distance >= float('inf') and self.left_backward_distance <= self.left_forward_distance and abs(self.back_l_distance - self.back_r_distance) > 0.1:
                self.state = 'searching_for_wall'
                message.angular.z = 0.0
            else:
                message.angular.z = -0.1
                message.linear.x = -0.018
        self.get_logger().info(self.state)
        self.cmdvel_publisher.publish(message)

def main(args=None):

    rclpy.init(args=args)
    turtle_follower = NavigateObstacles()

    rclpy.spin(turtle_follower)

    turtle_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

