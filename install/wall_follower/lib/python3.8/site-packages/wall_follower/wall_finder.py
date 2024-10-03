import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from custom_interfaces.srv import FindWall
from std_srvs.srv import Empty
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
class WallFind(Node):
    def __init__(self):
        super().__init__('wallfinder')
        self.reentrant_group_1=ReentrantCallbackGroup()
        self.srv_=self.create_service(FindWall,'find_wall', self.findwall_callback, callback_group=self.reentrant_group_1)
        self.publish_=self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_=self.create_subscription(LaserScan, '/scan', self.subscriber_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE), callback_group=self.reentrant_group_1)
        self._feedback=self.create_publisher(Bool, '/wallfound', 10)
        self.feedback=Bool()
    def subscriber_callback(self, msg : LaserScan):
        self.wall_distance=min(msg.ranges)
        self.front_distance=msg.ranges[360] #change to 360 in real robot it's 0 in gazebo
        self.right_distance=msg.ranges[220] #change to +220 in real robot it's -200 in gazebo
    def findwall_callback(self, request, response): 
        cmd = Twist()
        self.step=True
        self.process=True
        self.here=True
        rate = self.create_rate(60)  # 10 Hz 
        while(self.process==True and self.right_distance>0.3):
            if (self.process==False):
                break
            if (self.wall_distance+0.1 <= self.front_distance) and self.step==True:
                cmd.angular.z=1.5
                self.publish_.publish(cmd)
                self.get_logger().info("wall distace is: %r" % self.wall_distance)
                self.get_logger().info("front distace is: %r" % self.front_distance)
            else:
                    cmd.angular.z=0.0
                    self.publish_.publish(cmd)
                    self.step=False
                    if (self.front_distance>0.35 and self.here==True):
                        cmd.linear.x=0.1
                        self.publish_.publish(cmd)
                    elif(self.front_distance<0.3 and self.here==True):
                        cmd.linear.x=-0.1
                        self.publish_.publish(cmd)
                    elif(self.front_distance>=0.3 and self.front_distance<=0.35):
                        cmd.linear.x=0.0
                        self.publish_.publish(cmd)
                        self.here=False
                    if (self.right_distance>0.3 and self.here==False):
                        cmd.angular.z=1.0
                        self.publish_.publish(cmd)
                    elif(self.here==False):
                        cmd.angular.z=0.0
                        self.publish_.publish(cmd)
                        self.process=False
            rate.sleep()  # Prevent blocking other callbacks
        cmd.angular.z=0.0
        self.publish_.publish(cmd)
        response.wallfound=True
        self.feedback.data=True
        self._feedback.publish(self.feedback)
        return response


def main(args=None):
    rclpy.init(args=args)
    find_wall=WallFind()
    executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(find_wall)
    try :
        executor.spin()
    finally: 
        find_wall.destroy_node()
    rclpy.shutdown()
if __name__=='__main__':
    main()