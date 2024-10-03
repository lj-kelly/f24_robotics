'''matplotlib citation: https://matplotlib.org/stable/tutorials/index.html'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import matplotlib.pyplot as plt
from PIL import Image
import signal
import sys
import os



LINEAR_VEL = 0.1 #was 0.07
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.78
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90
START = 0
OSCILLATION_THRESHOLD = 6

class RandomWalk(Node):

    def __init__(self):
        super().__init__('walle_wall_follow')
        self.scan_cleaned = []
        self.stall = False
        self.backup = False
        self.oscillation_count = 0
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.START = 0
        self.initial_position = (5.2, 6)  
        self.previous_position = None
        self.total_distance = 0.0
        self.positions = [] 
        self.max_distance = 0.0  
        self.most_distant_point = None  
        self.moved_once = True
        


    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = []
    
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
            	self.scan_cleaned.append(reading)



    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        

        self.pose_saved=position

        self.get_logger().info(f"Total distance covered: {self.total_distance:.2f} meters")
        
        
        
        diffX = math.fabs(self.pose_saved.x- position.x)
        diffY = math.fabs(self.pose_saved.y - position.y)
        if (diffX < 0.0001 and diffY < 0.0001):
           self.stall = True
        else:
           self.stall = False
        
        if (diffX < 0.1 and diffY < 0.1):
           self.backup = True
        else:
           self.backup = False
           
        return None


    def timer_callback(self):

        if (len(self.scan_cleaned)==0):
    	    self.turtlebot_moving = False
    	    return
    	
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        #move forward one meter block
        if(self.total_distance < 1.0): #move fwd
            self.cmd.linear.x = 0.075 #speed is 75mm/s
            # self.cmd.linear.x = 0.15 #speed is 150mm/s
            self.cmd.angular.z = 0.0
            self.cmd.linear.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
        else: #stop
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.cmd.linear.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = False
            return




        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        if self.stall == True:
           self.get_logger().info('Stall reported')
           
        
        self.get_logger().info('Publishing: "%s"' % self.cmd)
 



def signal_handler(sig, frame):
    print("Termintaing Wall-E...")
    if 'walle_wall_follow' in globals():
        walle_wall_follow.print_trial_results()  
        walle_wall_follow.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):

    rclpy.init(args=args)

    global walle_wall_follow
    walle_wall_follow = RandomWalk()
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.spin(walle_wall_follow)




if __name__ == '__main__':
    main()