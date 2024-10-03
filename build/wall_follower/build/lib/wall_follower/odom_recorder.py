import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from custom_interfaces.action import OdomRecord
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np 
import time
import math
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

#apply multi threading callbacks
class OdomServer(Node):
    def __init__(self):
        super().__init__('odom_server')
        self.callback_group_1=MutuallyExclusiveCallbackGroup()
        self.callback_group_2=MutuallyExclusiveCallbackGroup()
        self.odom_server=ActionServer(self, OdomRecord, "record_odom",self.execute_callback, callback_group=self.callback_group_1)
        self.Odometry_subscriber=self.create_subscription(Odometry, "/odom", self.Odometry_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE), callback_group=self.callback_group_2)
        self.Assign_odom=self.create_timer(1.0, self.timer_callback, callback_group=self.callback_group_2)
        self.first_odom=Point()
        self.last_odom=Point()
        self.first_odom_assigned=False
        self.start_recording=False
        self.start_publishing=False
        self.last_x=Point().x
        self.last_y=Point().y
        self.odom_record=[]
        self.total_distance=0.0
        self.lap_distance=0.0
    def timer_callback(self):
        if self.start_recording==True:
            self.total_distance+=math.sqrt((self.last_x-self.last_odom.x)**2 + (self.last_y-self.last_odom.y)**2)
            self.last_odom.x=self.last_x
            self.last_odom.y=self.last_y         
            self.odom_record.append(Point(x=self.last_x, y=self.last_y))
            self.start_publishing=True
    def Odometry_callback(self, odometry: Odometry):
        if self.start_recording==True:
            if (self.first_odom_assigned==False and self.start_recording==True):
                self.first_odom.x=odometry.pose.pose.position.x
                self.first_odom.y=odometry.pose.pose.position.y
                self.first_odom_assigned=True
            self.last_x=odometry.pose.pose.position.x
            self.last_y=odometry.pose.pose.position.y
    def execute_callback(self, goal_handle):
        while rclpy.ok():
            self.start_recording=True
            feedback_msg=OdomRecord.Feedback()
            feedback_msg.current_total=self.total_distance
            goal_handle.publish_feedback(feedback_msg)
            self.lap_distance=math.sqrt((self.last_x-self.first_odom.x)**2+(self.last_y-self.first_odom.y)**2)
            if (self.lap_distance<=0.2 and self.total_distance>1.4):
                goal_handle.succeed()
                break
        self.start_recording=False
        result=OdomRecord.Result()
        result.list_of_odoms=self.odom_record
        return result
def main(args=None):
    rclpy.init(args=args)
    action_server=OdomServer()
    executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(action_server)
    try:
        executor.spin()
    finally:
        action_server.destroy_node()
    rclpy.shutdown()
if __name__=='__main__':
    main()