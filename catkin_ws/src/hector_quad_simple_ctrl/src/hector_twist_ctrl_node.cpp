#include <ros/ros.h>
// #include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/YawrateCommand.h>
#include <hector_uav_msgs/ThrustCommand.h>
#include <hector_uav_msgs/AttitudeCommand.h>
#include <hector_uav_msgs/TakeoffAction.h>
#include <hector_uav_msgs/LandingAction.h>
#include <hector_uav_msgs/PoseAction.h>
#include <hector_quadrotor_interface/limiters.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <actionlib/client/simple_action_client.h>

namespace hector_quadrotor
{

class Teleop
{
private:
  typedef actionlib::SimpleActionClient<hector_uav_msgs::LandingAction> LandingClient;
  typedef actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction> TakeoffClient;
  typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseClient;

  ros::NodeHandle node_handle_;
  ros::Subscriber vel_cmd_subscriber_;
  ros::Publisher velocity_publisher_, attitude_publisher_, yawrate_publisher_, thrust_publisher_;
  ros::ServiceClient motor_enable_service_;
  boost::shared_ptr<LandingClient> landing_client_;
  boost::shared_ptr<TakeoffClient> takeoff_client_;
  boost::shared_ptr<PoseClient> pose_client_;

  geometry_msgs::PoseStamped pose_;
  double yaw_;

  std::string base_link_frame_, base_stabilized_frame_, world_frame_;

public:
  Teleop()
  {
    ros::NodeHandle private_nh("~");

    // TODO dynamic reconfig
    std::string control_mode;
    private_nh.param<std::string>("control_mode", control_mode, "twist");

    ros::NodeHandle robot_nh;

    // TODO factor out
    robot_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    robot_nh.param<std::string>("world_frame", world_frame_, "world");
    robot_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");

    attitude_publisher_ = robot_nh.advertise<hector_uav_msgs::AttitudeCommand>("command/attitude", 10);
    yawrate_publisher_ = robot_nh.advertise<hector_uav_msgs::YawrateCommand>("command/yawrate", 10);
    thrust_publisher_ = robot_nh.advertise<hector_uav_msgs::ThrustCommand>("command/thrust",10);

    vel_cmd_subscriber_ = node_handle_.subscribe<geometry_msgs::Twist>("velocity_cmd", 1, boost::bind(&Teleop::VelCmdTwistCallback, this, _1));
    velocity_publisher_ = robot_nh.advertise<geometry_msgs::TwistStamped>("command/twist",10);

    motor_enable_service_ = robot_nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    takeoff_client_ = boost::shared_ptr<TakeoffClient>(new TakeoffClient(robot_nh, "action/takeoff"));
    landing_client_ = boost::shared_ptr<LandingClient>(new LandingClient(robot_nh, "action/landing"));
    pose_client_ = boost::shared_ptr<PoseClient>(new PoseClient(robot_nh, "action/pose"));

    }

  ~Teleop()
  {
    stop();
  }

  void VelCmdTwistCallback(const geometry_msgs::TwistConstPtr &vel_msg)
  {
    geometry_msgs::TwistStamped velocity;
    velocity.header.frame_id = base_stabilized_frame_;
    velocity.header.stamp = ros::Time::now();

    velocity.twist.linear.x = vel_msg->linear.x;
    velocity.twist.linear.y = vel_msg->linear.y;
    velocity.twist.linear.z = vel_msg->linear.z;
    velocity.twist.angular.z = vel_msg->angular.z;
   
    velocity_publisher_.publish(velocity);

   
  }

  bool enableMotors(bool enable)
  {
    if (!motor_enable_service_.waitForExistence(ros::Duration(5.0)))
    {
      ROS_WARN("Motor enable service not found");
      return false;
    }

    hector_uav_msgs::EnableMotors srv;
    srv.request.enable = enable;
    return motor_enable_service_.call(srv);
  }

  void stop()
  {
    if (velocity_publisher_.getNumSubscribers() > 0)
    {
      velocity_publisher_.publish(geometry_msgs::TwistStamped());
    }
    if (attitude_publisher_.getNumSubscribers() > 0)
    {
      attitude_publisher_.publish(hector_uav_msgs::AttitudeCommand());
    }
    if (thrust_publisher_.getNumSubscribers() > 0)
    {
      thrust_publisher_.publish(hector_uav_msgs::ThrustCommand());
    }
    if (yawrate_publisher_.getNumSubscribers() > 0)
    {
      yawrate_publisher_.publish(hector_uav_msgs::YawrateCommand());
    }
  }
};

} // namespace hector_quadrotor

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quadrotor_teleop");

  hector_quadrotor::Teleop teleop;
  ros::spin();

  return 0;
}
