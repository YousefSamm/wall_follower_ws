import rclpy
from rclpy.node import Node
from geometry_msgs.mgs import Twist
from sensor_msgs.msg import LaserScan
import time
from rclpy.qos import QoSReliabilityPolicy, QoSProfile
class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.publisher_=self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_=self.create_subscription(LaserScan, '/scan', self.Laser_callback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))
    def Laser_callback(self, msg: LaserScan):
        cmd=Twist()
        distance=msg.ranges[90]
        if distance>=0.2 or distance<=0.3:
            cmd.linear.x=0.5
            cmd.angular.z=0.0
            self.publisher_.publish(cmd)
        elif distance > 0.3:
            cmd.linear.x=0.5
            cmd.angular.z=-0.1
            self.publisher_.publish(cmd)
        else:
            cmd.linear.x=0.5
            cmd.angular.z=0.1
            self.publisher_.publish(cmd)







def main(args=None):
    rclpy.init(args=args)
    node=WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__=='__main__':
    main()