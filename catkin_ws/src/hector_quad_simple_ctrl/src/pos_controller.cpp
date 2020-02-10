#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
using namespace std;

namespace hector_quadrotor
{
class Teleop
{
  private:
    geometry_msgs::Twist twist_cmd;
    geometry_msgs::Twist pos_current;
    geometry_msgs::Twist pos_target;
    geometry_msgs::Point euler_current;
    geometry_msgs::Quaternion quaternion_current;
    double yaw;
    double p_xy, p_z, p_yaw;
    
    ros::NodeHandle nh_;
    ros::Subscriber get_target_goal, get_current_pos;
    ros::Publisher pub_twist_cmd;

  public:
    
    Teleop(ros::NodeHandle& nh):nh_(nh)
    {
        ros::NodeHandle pnh("~");
        // 
        pnh.param("p_xy", p_xy, 0.20);
        pnh.param("p_z", p_z, 0.20);
        pnh.param("p_yaw", p_yaw, 0.20);

        get_target_goal = nh_.subscribe("pose_cmd", 1, &Teleop::PositionControllerCB,this);
        get_current_pos = nh_.subscribe("ground_truth/state", 1, &Teleop::CurrentPositionCB,this);
        pub_twist_cmd = nh_.advertise<geometry_msgs::Twist>("velocity_cmd", 1);
        ros::spin();
    }

    ~Teleop()
    {
    }

    void Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Point &euler)
    {
        double q0 = quat.w;
        double q1 = quat.x;
        double q2 = quat.y;
        double q3 = quat.z;
        double roll, pitch, yaw;
        double t0 = -2.0 * (q2 * q2 + q3 * q3) + 1.0;
        double t1 = +2.0 * (q1 * q2 + q0 * q3);
        double t2 = -2.0 * (q1 * q3 - q0 * q2);
        double t3 = +2.0 * (q2 * q3 + q0 * q1);
        double t4 = -2.0 * (q1 * q1 + q2 * q2) + 1.0;

        roll = -atan2(t3, t4);
        pitch = -asin(t2);
        yaw = atan2(t1, t0);
        // the xyz-axis of camera is a little different from the xyz-axis of body. from cam_frame to body_frame
        // 0 -1  0
        // 1  0  0
        // 0  0  1
        euler.x = -pitch;
        euler.y = roll;
        euler.z = yaw;
    }

    void PositionControllerCB(const geometry_msgs::Twist &msg)
    {
        pos_target = msg;
        double e_x,e_y,e_z,e_yaw; 
        double vx_n, vy_n; //reference coordinate frame
        vx_n = p_xy * (pos_target.linear.x - pos_current.linear.x);
        vy_n = p_xy * (pos_target.linear.y - pos_current.linear.y);
        twist_cmd.linear.x = cos(yaw) * vx_n + sin(yaw) * vy_n;
        twist_cmd.linear.y = -sin(yaw) * vx_n + cos(yaw) * vy_n;
        twist_cmd.linear.z = p_z * (pos_target.linear.z - pos_current.linear.z);
        twist_cmd.angular.z = p_yaw * (pos_target.angular.z - pos_current.angular.z);
        pub_twist_cmd.publish(twist_cmd);
    }
    void CurrentPositionCB(const nav_msgs::Odometry &msg)
    {

        pos_current.linear.x = msg.pose.pose.position.x;
        pos_current.linear.y = msg.pose.pose.position.y;
        pos_current.linear.z = msg.pose.pose.position.z;
        quaternion_current = msg.pose.pose.orientation;
        Quat2Euler(quaternion_current, euler_current);
        pos_current.angular.z = euler_current.z;
        yaw = pos_current.angular.z;
       
    }
};

} // namespace hector_quadrotor

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_target_goal");
    ros::NodeHandle nh;
    hector_quadrotor::Teleop teleop(nh);
    

    return 0;
}
