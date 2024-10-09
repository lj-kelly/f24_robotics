import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import signal
import sys

# Linear speed in meters/second (150 mm/s)
LINEAR_VEL = 0.15
TARGET_DISTANCE = 5.0  # Target distance in meters

class MoveRobot(Node):

   def __init__(self):
       super().__init__('move_robot')
       self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
       self.subscriber = self.create_subscription(
           Odometry,
           '/odom',
           self.odometry_callback,
           QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

       self.initial_position = None
       self.total_distance = 0.0

       # Timer for sending velocity commands
       timer_period = 0.1  # 100 ms
       self.timer = self.create_timer(timer_period, self.timer_callback)
       self.cmd = Twist()

   def odometry_callback(self, msg):
       current_position = msg.pose.pose.position

       # Save initial position when callback is first triggered
       if self.initial_position is None:
           self.initial_position = current_position

       # Calculate distance traveled
       dx = current_position.x - self.initial_position.x
       dy = current_position.y - self.initial_position.y
       self.total_distance = math.sqrt(dx**2 + dy**2)

       # Log odometry data and total distance
       self.get_logger().info(f"Total distance covered: {self.total_distance:.3f} meters")

   def timer_callback(self):
       if self.total_distance < TARGET_DISTANCE:
           # Move forward at 150 mm/s
           self.cmd.linear.x = LINEAR_VEL
           self.cmd.angular.z = 0.0
       else:
           # Stop once 5 meters is reached
           self.cmd.linear.x = 0.0
           self.get_logger().info("Target distance of 5 meters reached. Stopping the robot.")
       
       self.publisher_.publish(self.cmd)


def signal_handler(sig, frame):
   print("Terminating the robot...")
   if 'move_robot_node' in globals():
       move_robot_node.destroy_node()
   rclpy.shutdown()
   sys.exit(0)


def main(args=None):
   rclpy.init(args=args)

   global move_robot_node
   move_robot_node = MoveRobot()
   signal.signal(signal.SIGINT, signal_handler)

   rclpy.spin(move_robot_node)


if __name__ == '__main__':
   main()

