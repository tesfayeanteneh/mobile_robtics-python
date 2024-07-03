import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations

class LeaderControlNode(Node):
    def __init__(self):
        super().__init__('leader_control_node')
        self.cmd_pub = self.create_publisher(Twist, '/leader_robot/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/leader/odom', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def timer_callback(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        move_cmd.angular.z = 0.0
        self.cmd_pub.publish(move_cmd)

        # Simulate Odometry Data
        self.x += move_cmd.linear.x * 0.1
        self.th += move_cmd.angular.z * 0.1

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        odom.twist.twist.linear.x = move_cmd.linear.x
        odom.twist.twist.angular.z = move_cmd.angular.z
        self.odom_pub.publish(odom)

   def main(args=None):
       rclpy.init(args=args)
       node = LeaderControlNode()
       rclpy.spin(node)
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

