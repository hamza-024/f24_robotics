import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower_node')
        self.scan_cleaned = []
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
        self.cmd = Twist()
        self.following_wall = False
        self.timer = self.create_timer(0.5, self.timer_callback)

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
        pass  # Position tracking is removed as it is not relevant for this task.

    def timer_callback(self):
        if len(self.scan_cleaned) == 0:
            self.turtlebot_moving = False
            return

        # Find the minimum distances in the relevant LIDAR slices
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        # Obstacle in front, initiate wall following
        if front_lidar_min < SAFE_STOP_DISTANCE:
            self.cmd.linear.x = 0.0  # Stop moving forward
            self.cmd.angular.z = 0.3  # Turn left to start following the wall
            self.following_wall = True
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Obstacle detected, initiating wall following')

        # Continue wall following if the robot is avoiding an obstacle
        elif self.following_wall:
            if front_lidar_min >= LIDAR_AVOID_DISTANCE:
                # No obstacle in front, stop wall following
                self.following_wall = False
            else:
                # Adjust to follow the wall
                self.cmd.linear.x = 0.07  # Move forward slowly
                if right_lidar_min > left_lidar_min:
                    self.cmd.angular.z = -0.3  # Turn slightly right
                else:
                    self.cmd.angular.z = 0.3  # Turn slightly left
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Following the wall to avoid obstacle')

        # No obstacles, continue moving forward
        else:
            self.cmd.linear.x = 0.3  # Move forward
            self.cmd.angular.z = 0.0  # Move straight
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Path is clear, moving forward')

        self.get_logger().info(f'Obstacle distance: {front_lidar_min:.2f}')
        self.get_logger().info('Publishing: %s' % self.cmd)

def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollower()
    rclpy.spin(wall_follower_node)
    wall_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
