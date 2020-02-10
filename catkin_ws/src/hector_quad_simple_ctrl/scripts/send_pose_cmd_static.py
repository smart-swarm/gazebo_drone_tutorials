#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Header
from geometry_msgs.msg import Twist

pose_vel_ctrl1 = Twist()
# pose_vel_ctrl2 = Twist()
def drones_fly_goal():
    rospy.init_node('drones_fly_goal', anonymous=True)
    pub_pose_cmd1 = rospy.Publisher("/uav1/pose_cmd", Twist, queue_size=100)
    # pub_pose_cmd2 = rospy.Publisher("/uav2/pose_cmd", Twist, queue_size=100)
    i = 0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pose_vel_ctrl1.linear.z = 2
        pose_vel_ctrl1.angular.x = 0.0
        pose_vel_ctrl1.angular.y = 0.0
        pose_vel_ctrl1.angular.z = 0.0
        # pose_vel_ctrl2.linear.z = 2
        # pose_vel_ctrl2.angular.x = 0.0
        # pose_vel_ctrl2.angular.y = 0.0
        # pose_vel_ctrl2.angular.z = 0.0
        if (i/400)%2 == 0:
            pose_vel_ctrl1.linear.x = -3.5
            pose_vel_ctrl1.linear.y = -3.0
        else:
            pose_vel_ctrl1.linear.x = 3
            pose_vel_ctrl1.linear.y = 3
        pub_pose_cmd1.publish(pose_vel_ctrl1)
        i=i+1
        # pub_pose_cmd2.publish(pose_vel_ctrl2)
        rate.sleep()

if __name__ == '__main__':
    
    # drone = str(sys.argv[1])
    # X = float(sys.argv[2])
    # Y = float(sys.argv[3])
    # Z = float(sys.argv[4])
    # Yaw = float(sys.argv[5])
    drones_fly_goal()