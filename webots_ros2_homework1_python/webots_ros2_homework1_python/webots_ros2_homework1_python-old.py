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



LINEAR_VEL = 0.09 #was 0.07
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
        self.initial_position = (1.8, 6.3)  
        self.previous_position = None
        self.total_distance = 0.0
        self.positions = [] 
        self.max_distance = 0.0  
        self.most_distant_point = None  
        


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
        
        normalized_x = position.x + self.initial_position[0]
        normalized_y = position.y + self.initial_position[1]

        if self.previous_position is not None:
            dx = normalized_x - self.previous_position[0]
            dy = normalized_y - self.previous_position[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            self.total_distance += distance
        

        self.positions.append((normalized_x, normalized_y))

        distance_from_start = math.sqrt(
            (normalized_x - self.initial_position[0]) ** 2 +
            (normalized_y - self.initial_position[1]) ** 2
        )


        if distance_from_start > self.max_distance:
            self.max_distance = distance_from_start
            self.most_distant_point = (normalized_x, normalized_y)

        self.previous_position = (normalized_x, normalized_y)
        

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

        if front_lidar_min < SAFE_STOP_DISTANCE: #stop section
            if self.turtlebot_moving == True:
                self.cmd.linear.x = 0.0 
                self.cmd.angular.z = 0.0 
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.get_logger().info('Stopping')
                return
        elif front_lidar_min < LIDAR_AVOID_DISTANCE: #turn section
                self.cmd.linear.x = 0.0
                if (right_lidar_min < left_lidar_min):
                    #turn left
                   self.cmd.angular.z = 0.29
                   self.oscillation_count += 1
                else:
                    #turn right
                   self.cmd.angular.z = -0.29
                   self.oscillation_count += 1
                if self.oscillation_count > 6: #stop oscillating if stuck
                    self.cmd.angular.z = 0.46 #was 48
                    self.cmd.linear.x = 0.00 #maybe->NO
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Turning')
                self.turtlebot_moving = True
                self.START = 1
        elif (self.START == 1) and (front_lidar_min > LIDAR_AVOID_DISTANCE): #wall follow, go forward and turn right a little bit
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = -0.16
            # self.cmd.linear.z = 0.0
            self.oscillation_count = 0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True

        elif self.backup == True and self.START == 1: #reverse and turn a little bit if stuck
            self.cmd.linear.x = -0.7
            self.cmd.angular.z = 0.3
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True

        else: #go straight
            self.cmd.linear.x = 0.12
            self.cmd.angular.z = 0.0
            self.cmd.linear.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            
        


        if self.stall == True and self.turtlebot_moving == False: #reverse if stuck
            self.cmd.linear.x = -0.6
            self.cmd.linear.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
        


        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        if self.stall == True:
           self.get_logger().info('Stall reported')
           
        
        self.get_logger().info('Publishing: "%s"' % self.cmd)
 
    def save_positions_to_file(self, filename="/home/liam/f24_robotics/webots_ros2_homework1_python/robot_path.csv"):
        """Save the robot's path to a CSV file for later plotting."""
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, "w") as f:
            for position in self.positions:
                f.write(f"{position[0]},{position[1]}\n")


    def plot_path(self):
        """Plot the robot's path using matplotlib on top of a background image."""
        img_path = '/home/liam/f24_robotics/webots_ros2_homework1_python/resource/apartment2.jpg'
    
        if not os.path.exists(img_path):
            self.get_logger().error(f"Image file not found: {img_path}")
            return

        img = Image.open(img_path)
        rotated_positions = [(x, y) for x, y in self.positions]
        
        img_width, img_height = img.size

        fig, ax = plt.subplots()

        ax.imshow(img, extent=[-1, 11.5, -.5, 10])

        x_vals = [pos[0] for pos in rotated_positions]
        y_vals = [pos[1] for pos in rotated_positions]

        ax.plot(x_vals, y_vals, marker='o', color='blue', linewidth=2)

        ax.set_title("Robot Path Overlaid on Apartment Layout")
        ax.set_xlabel("X Position (meters)")
        ax.set_ylabel("Y Position (meters)")

        plt.show()

    def print_trial_results(self):
        """Print the results of the trial, including total distance and most distant point."""
        self.get_logger().info(f"Trial results:")
        self.get_logger().info(f"Total distance: {self.total_distance:.2f} meters")
        if self.most_distant_point:
            self.get_logger().info(f"Most distant point: {self.most_distant_point} at distance {self.max_distance:.2f} meters")
        else:
            self.get_logger().info("No distant point recorded")


def signal_handler(sig, frame):
    print("Termintaing Wall-E...")
    if 'walle_wall_follow' in globals():
        walle_wall_follow.save_positions_to_file()
        walle_wall_follow.plot_path()
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
