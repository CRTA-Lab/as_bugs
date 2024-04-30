import math
import rclpy
from rclpy.node import Node
import tf_transformations as transform
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

class Bug0(Node):
    #Initialization
    def __init__(self):
        super().__init__('bugx')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.bug_algorithm_timer = self.create_timer(0.1, self.bug_algorithm_callback)
        self.goal_x = None
        self.goal_y = None
        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.fl_sensor_value = 0.0
        self.fr_sensor_value = 0.0
        self.cmd_vel_msg = Twist()

    #Method for goal update
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    #Method for robot current position
    def location_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        self.current_theta = transform.euler_from_quaternion(q)[2] #[-pi, pi]

    #Methods for updating sensor values
    def fl_sensor_callback(self, msg):
        self.fl_sensor_value = msg.range

    def fr_sensor_callback(self, msg):
        self.fr_sensor_value = msg.range

    def bug_algorithm_callback(self):
        print("Current X: " , self.current_x)
        print("Current Y: " , self.current_y)
        print("Current theta: " , self.current_theta)
        print("FL Sensor: " , self.fl_sensor_value)
        print("FR Sensor: " , self.fr_sensor_value)
        print("Goal X: " , self.goal_x)
        print("Goal Y: " , self.goal_y)

            
def main(args=None):

    rclpy.init(args=args)
    bug_node = Bug0()
    rclpy.spin(bug_node)
    bug_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()