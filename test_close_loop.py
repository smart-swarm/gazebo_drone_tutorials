#!/usr/bin/env python
# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib
import rospy
import math

from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from std_msgs.msg import Header  
from std_msgs.msg import String 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
class DroneInterface:
    # _instance = None
    # def __new__(cls, *args, **kwargs):
    # 	if not PlatformInterface._instance:
    # 		PlatformInterface._instance = super(PlatformInterface, cls).__new__(cls, *args, **kwargs)
    # 	return PlatformInterface._instance

    def __init__(self, navdata_name, cmd_topic_name):
        self.local_network = 1
        self.k_p_xy = 0.5
        self.k_p_z = 0.3
        self.k_d_xy = 0.3
        self.k_d_z = 0.3
        self.k_yaw = 0.5
        self.limit_cmd_xy = 10
        self.target_point = Twist()
        self.state = Odometry()
        self.move_cmd = Twist()
        self.get_target = False
        self.sub_pose = rospy.Subscriber(navdata_name, Odometry, self.callback_pose)
        self.pub_move_cmd = rospy.Publisher(cmd_topic_name, Twist, queue_size=1)
        rospy.on_shutdown(self.Reset)

    # def ReceiveNavdata(self,navdata):
    # 	# Although there is a lot of data in this packet, we're only interested in the state at the moment
    # 	self.status = navdata.state
    def Reset(self):
        print(" shut down the ros...")

    def callback_pose(self, msg):
        self.state = msg

    def move_cmd_send(self, move_cmd):
        self.pub_move_cmd.publish(move_cmd)

    def set_target_point(self, point):
        self.target_point = point
        self.get_target = True

    def quat2eul(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
            
        t0 = -2.0 * (y * y + z * z) + 1.0
        t1 = +2.0 * (x * y + w * z)
        t2 = -2.0 * (x * z - w * y)
        t3 = +2.0 * (y * z + w * x)
        t4 = -2.0 * (x * x + y * y) + 1.0
        euler_angle = Point()
        euler_angle.x = -math.asin(t2)
        euler_angle.y = -math.atan2(t3, t4)
        euler_angle.z = -math.atan2(t1, t0)
        return euler_angle

    def run(self):
        cmd = Twist()
        if (self.get_target):
        	err_x = (self.target_point.linear.x - self.state.pose.pose.position.x)
        	err_y = (self.target_point.linear.y - self.state.pose.pose.position.y)
        	euler_angle = self.quat2eul(self.state.pose.pose.orientation)
            cmd.linear.x = self.k_p_xy * (err_x * math.cos(euler_angle) + err_y * math.sin(euler_angle))
            cmd.linear.y = self.k_p_xy * (- err_x * math.sin(euler_angle) + err_y * math.cos(euler_angle))
            cmd.linear.z = self.k_p_z * (self.target_point.linear.z - self.state.pose.pose.position.z)
            
            cmd.angular.z = self.k_yaw * (self.target_point.angular.z - euler_angle.z)
        self.move_cmd_send(cmd)
        

if __name__ == "__main__":
    rospy.init_node('DroneInterface1')
    drone_1 = DroneInterface('/uav1/ground_truth/state', '/A/uav1/velocity_cmd')
    sub_target1 = rospy.Subscriber('/uav1/target_point', drone_1.set_target_point)
    target = Twist()
    target.linear.x = 5
    target.linear.y = 4
    target.linear.z = 2
    target.angular.z = 0
    while not rospy.is_shutdown():
        drone_1.set_target_point(target)
        drone_1.run()







