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
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.distance = 999
        self.threshold = self.get_parameter('distance_threshold').value
        self.turn_cw = self.get_parameter('clockwise_turn').value
        self.state = "searching_for_wall"
        self.counter = 0
        self.distance_array = []
        self.data = []
        # self.alive_turtles_pub = self.create_publisher(Float64MultiArray, '/alive_turtles', 10) #not used
        # self.pose_subscriber = self.create_subscription(Pose, 'turtlesim1/turtle1/pose', self.update_pose, 10)
        # self.pose = Pose()
        # self.rate = self.create_rate(10)

        # # Create a client to call the TeleportAbsolute service
        # self.teleport_client = self.create_client(TeleportAbsolute, 'turtlesim1/turtle1/teleport_absolute')
        # self.spawn_client = self.create_client(Spawn, 'turtlesim1/spawn')
        # self.kill_client = self.create_client(Kill, 'turtlesim1/kill')
        # self.set_pen_client = self.create_client(SetPen, 'turtlesim1/turtle1/set_pen')
        # self.catch_turtle_service = self.create_service(Kill, 'turtlesim1/catch_turtle', self.catch_turtle_callback)

        self.scan_subscriber = self.create_subscription(Scan, '/scan', self.print_scan, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.distance_to_wall = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.turn_threshold = 0.1

    def scan_callback(self, data):
        # Update distances to obstacles
        self.distance_to_wall = data.ranges[len(data.ranges)//2]
        self.left_distance = min(data.ranges[:len(data.ranges)//2])
        self.right_distance = min(data.ranges[len(data.ranges)//2:])

    def print_scan(self, data):
        self.data = data

        forward_index = 360

        right_index = 540

        left_index = 180

        self.get_logger().info("forward %s" % str(data.ranges[forward_index]))
        self.get_logger().info("right %s" % str(data.ranges[right_index]))

        new_distance = data.ranges[forward_index]

        if not new_distance == float('inf'):

            self.distance = new_distance

        new_distance = data.ranges[right_index]

        if not new_distance == float('inf'):

            self.right_distance = new_distance

        new_distance = data.ranges[left_index]

        if not new_distance == float('inf'):

            self.left_distance = new_distance

    def timer_callback(self):

        message = Twist()

        
        if self.state == 'searching_for_wall':
        	if self.distance < self.threshold:
        		self.state = 'rotating_after_found_wall'
        		self.counter = 5
        	message.angular.z = 0.0
        	message.linear.x = 0.5

        # if self.state == 'rotating_after_found_wall':
        # 	if self.counter <= 0:
        # 		self.state = 'following_wall'
        # 	message.linear.x = 0.0
        # 	message.angular.z = math.pi/2
        # 	self.counter -= 1


        if self.state == 'rotating_after_found_wall':
            angle_to_wall = math.atan2(self.right_distance - self.left_distance, 1.0)

            if abs(angle_to_wall) > self.turn_threshold:
                message.angular.z = angle_to_wall
            else:
                # Stop turning when aligned with the wall
                self.state = 'following_wall'
                message.angular.z = 0.0
            

        # if self.state == 'rotating_after_found_wall':
        #     dist = self.data.ranges[175:184]
        #     angles = 88
        #     dist_updated = []
            


        #     for x in dist:
        #         if (angles <= 92):
        #             y = math.cos((math.pi/2) - (angles* math.pi/180))
        #             dist_updated.append(x/y)
        #             angles += 0.5
        #         else:
        #             break

        #     (slope, std_dev) = self.find_line_slope_and_std(dist_updated)
        #     if (abs(slope) > 0.5 or std_dev > 0.15):
        #         message.linear.x = 0.0
        #         message.angular.z = math.pi/2
        #     else:
        #         message.angular.z = 0.0
        #         self.state = 'following_wall'

            


            #self.get_logger().info("dist_updated %s" % str(dist_updated))


        if self.state == 'following_wall':
        	if self.right_distance > self.threshold + 1.5:
        		self.state = 'rotating_after_following_wall'
        		self.counter = 5

        	message.linear.x = 0.5
        	message.angular.z = 0.0

        if self.state == 'rotating_after_following_wall':
        	if self.counter <= 0:
        		self.state = 'searching_for_wall'
        	message.linear.x = 0.0
        	message.angular.z = math.pi/2
        	self.counter -= 1

        self.get_logger().info(self.state)

       	self.cmdvel_publisher.publish(message)


    def find_line_slope_and_std(self, y_values):
        n = len(y_values)
        x_values = list(range(n))
        sum_x = sum(x_values)
        sum_y = sum(y_values)
        sum_xy = sum(x * y for x, y in zip(x_values, y_values))
        sum_x2 = sum(x**2 for x in x_values)

        # Calculate the slope
        numerator = n * sum_xy - sum_x * sum_y
        denominator = n * sum_x2 - sum_x**2
        if denominator == 0:
            return None, None  # Avoid division by zero

        slope = numerator / denominator
        intercept = (sum_y - slope * sum_x) / n

        # Calculate residuals and their standard deviation
        residuals = [(y - (slope * x + intercept)) for x, y in zip(x_values, y_values)]
        mean_residual = sum(residuals) / n
        variance = sum((res - mean_residual)**2 for res in residuals) / (n - 1)
        std_dev = variance ** 0.5



        return slope, std_dev

def main(args=None):

    rclpy.init(args=args)
    turtle_follower = NavigateObstacles()

    rclpy.spin(turtle_follower)

    turtle_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()