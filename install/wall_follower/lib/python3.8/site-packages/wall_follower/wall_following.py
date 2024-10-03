import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import time
from custom_interfaces.srv import FindWall
from std_srvs.srv import Empty
from rclpy.qos import QoSReliabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from custom_interfaces.action import OdomRecord
class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.callbackgroup_1= MutuallyExclusiveCallbackGroup()
        self.callbackgroup_2=MutuallyExclusiveCallbackGroup()
        self._action_client=ActionClient(self,OdomRecord,"record_odom", callback_group=self.callbackgroup_2)
        self.publisher_=self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_=self.create_subscription(LaserScan, '/scan', self.Laser_callback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE), callback_group=self.callbackgroup_1)
        self.subscriber_=self.create_subscription(Bool, '/wallfound', self._feedback_callback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),callback_group=self.callbackgroup_2)
        self.client=self.create_client(FindWall, 'find_wall')
        self.wallfoundbool=Bool.data #consider adding Bool() instead of Bool
        self.goal_sent=False
        while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
        self.req=FindWall.Request()
    def _feedback_callback(self, msg: Bool):
        self.wallfoundbool=msg.data
    def send_request(self):
        self.future=self.client.call_async(self.req)
    def send_goal(self):
        goal_msg=OdomRecord.Goal()
        self._action_client.wait_for_server()      
        self._send_goal_future=self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle=future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected :(')
            return
        self.get_logger().info('Goal Accpeted :)')
        self._get_result_future=goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    def get_result_callback(self, future):
        result=future.result()
        self.get_logger().info('Result: %s' % str(result))
    def feedback_callback(self, feedback_msg):
        feedback=feedback_msg
        self.get_logger().info('Received feedback: %s' % str(feedback))
        time.sleep(1.0)

    def Laser_callback(self, msg: LaserScan):
        cmd=Twist()
        right_distance=msg.ranges[-90]
        front_distance=msg.ranges[360]  #change to 360 in real robot and it's 0 in Gazebo
        if self.wallfoundbool:
            if not self.goal_sent:
                self.send_goal()
                self.goal_sent=True
            if (right_distance>=0.3 or right_distance<=0.35) and front_distance>0.5:
                cmd.linear.x=0.2
                cmd.angular.z=0.0
                self.publisher_.publish(cmd)
                if right_distance>0.34:
                    cmd.angular.z=-0.08
                    self.publisher_.publish(cmd)
                elif right_distance<0.3:
                    cmd.angular.z=+0.1
                    self.publisher_.publish(cmd)
                else:
                    cmd.angular.z=0.0
                    self.publisher_.publish(cmd)
            else:
                    cmd.linear.x=0.2
                    cmd.angular.z=2.5
                    self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node=WallFollower()
    executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    node.send_request()
    while rclpy.ok():
        if node.future.done():
            try:
                response=node.future.result()
                
            except Exception as e:
                node.get_logger.info('Service call Failed %r' % (e,))
        else:
            node.get_logger().info('the robot is going to the wall')
        break
    try:
        executor.spin()
    finally:
        node.destroy_node()
    rclpy.shutdown()
if __name__=='__main__':
    main()