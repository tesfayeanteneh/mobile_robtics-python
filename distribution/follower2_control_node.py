import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Follower2ControlNode(Node):
    def __init__(self):
        super().__init__('follower2_control_node')
        self.follower1_odom_sub = self.create_subscription(Odometry, '/follower1/odom', self.follower1_odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/follower2_robot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.follower1_odom = None

    def follower1_odom_callback(self, msg):
        self.follower1_odom = msg

    def timer_callback(self):
        if self.follower1_odom is not None:
            move_cmd = Twist()
            move_cmd.linear.x = self.follower1_odom.twist.twist.linear.x - 0.1  # Adjust to maintain distance
            move_cmd.angular.z = 0.0
            self.cmd_pub.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Follower2ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

