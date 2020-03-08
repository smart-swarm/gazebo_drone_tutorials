#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
import time
target_x = 19
target_y = 19
start_x = -20
start_y = 19
uav_start_time = 0
car_start_time = 0
uav_cost_time = 0
car_cost_time = 0
uav_finish = False
car_finish = False

def all_task_finished():
    print("All the task completed successfully.")

def pose_uav_CB(data):
    # print(data)
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    dist_end = math.sqrt((x-target_x)*(x-target_x)+(y-target_y)*(y-target_y))
    dist_start = math.sqrt((x - start_x) * (x - start_x) + (y - start_y) * (y - start_y))
    z = data.pose.pose.position.z
    if (z > 1.0 and dist_start < 1.0):
        uav_start_time = time.time()
    if (z < 5.0 and dist_end < 1.0):
        rospy.loginfo(rospy.get_caller_id() + " Congratulations! The uav solved the maze successfully.")
        uav_finish = True
        if (uav_start_time > 0):
            uav_cost_time = time.time() - uav_start_time
            rospy.loginfo(rospy.get_caller_id() + " Time Cost: " + uav_cost_time)
def pose_car_CB(data):
    # print(data)
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    dist_end = math.sqrt((x-target_x)*(x-target_x)+(y-target_y)*(y-target_y))
    dist_start = math.sqrt((x - start_x) * (x - start_x) + (y - start_y) * (y - start_y))
    if (dist_start < 1.0):
        car_start_time = time.time()
    if (dist_end < 1.0):
        rospy.loginfo(rospy.get_caller_id() + " Congratulations! The car solved the maze successfully.")
        car_finish = True
        if (car_start_time > 0):
            car_cost_time = time.time() - car_start_time
            rospy.loginfo(rospy.get_caller_id() + " Time Cost: " + car_cost_time)
def judge():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('sub_pose_test', anonymous=True)
    rospy.Subscriber("/uav1/ground_truth/state", Odometry, pose_uav_CB)
    rospy.Subscriber("/mobile_base/mobile_base_controller/odom", Odometry, pose_car_CB)
    if (uav_finish and car_finish):
        rospy.loginfo(rospy.get_caller_id() + " Sum of cost time is " + (car_cost_time + uav_cost_time))
        rospy.on_shutdown(all_task_finished)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    judge()
