
#include <ros/ros.h>
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
  ros::Subscriber cmd_subscriber_;//joy_subscriber_;
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
    private_nh.param<std::string>("control_mode", control_mode, "pose");

    ros::NodeHandle robot_nh;

    // TODO factor out
    robot_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    robot_nh.param<std::string>("world_frame", world_frame_, "world");
    robot_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");

    if (control_mode == "attitude")
    {
     
      cmd_subscriber_ = node_handle_.subscribe<geometry_msgs::Twist>("attitude_cmd", 1,
                                                                 boost::bind(&Teleop::cmdAttitudeCallback, this, _1));
      attitude_publisher_ = robot_nh.advertise<hector_uav_msgs::AttitudeCommand>(
          "command/attitude", 10);
      yawrate_publisher_ = robot_nh.advertise<hector_uav_msgs::YawrateCommand>(
          "command/yawrate", 10);
      thrust_publisher_ = robot_nh.advertise<hector_uav_msgs::ThrustCommand>("command/thrust",
                                                                                 10);
    }
    else if (control_mode == "velocity")
    {
     
      cmd_subscriber_ = node_handle_.subscribe<geometry_msgs::Twist>("velocity_cmd", 1,
                                                                 boost::bind(&Teleop::cmdTwistCallback, this, _1));
      velocity_publisher_ = robot_nh.advertise<geometry_msgs::TwistStamped>("command/twist",
                                                                                10);
    }
    else if (control_mode == "position")
    {

      cmd_subscriber_ = node_handle_.subscribe<geometry_msgs::Pose>("pose_cmd", 1,
                                                                 boost::bind(&Teleop::cmdPoseCallback, this, _1));

      pose_.pose.position.x = 0;
      pose_.pose.position.y = 0;
      pose_.pose.position.z = 0;
      pose_.pose.orientation.x = 0;
      pose_.pose.orientation.y = 0;
      pose_.pose.orientation.z = 0;
      pose_.pose.orientation.w = 1;
    }
    else
    {
      ROS_ERROR_STREAM("Unsupported control mode: " << control_mode);
    }

    motor_enable_service_ = robot_nh.serviceClient<hector_uav_msgs::EnableMotors>(
        "enable_motors");
    takeoff_client_ = boost::shared_ptr<TakeoffClient>(new TakeoffClient(robot_nh, "action/takeoff"));
    landing_client_ = boost::shared_ptr<LandingClient>(new LandingClient(robot_nh, "action/landing"));
    pose_client_ = boost::shared_ptr<PoseClient>(new PoseClient(robot_nh, "action/pose"));

  }

  ~Teleop()
  {
    stop();
  }

  void cmdAttitudeCallback(const geometry_msgs::TwistConstPtr &msg)
  {

    hector_uav_msgs::AttitudeCommand attitude;
    hector_uav_msgs::ThrustCommand thrust;
    hector_uav_msgs::YawrateCommand yawrate;

    attitude.header.stamp = thrust.header.stamp = yawrate.header.stamp = ros::Time::now();
    attitude.header.frame_id = yawrate.header.frame_id = base_stabilized_frame_;
    thrust.header.frame_id = base_link_frame_;

    attitude.roll = msg->angular.x;
    attitude.pitch = msg->angular.y;

    attitude_publisher_.publish(attitude);

    thrust.thrust = msg->linear.z;
    thrust_publisher_.publish(thrust);

    yawrate.turnrate = msg->angular.z;

    yawrate_publisher_.publish(yawrate);

  }

  void cmdTwistCallback(const geometry_msgs::TwistConstPtr &msg)
  {
    geometry_msgs::TwistStamped velocity;
    velocity.header.frame_id = base_stabilized_frame_;
    velocity.header.stamp = ros::Time::now();

    velocity.twist.linear.x = msg->linear.x;
    velocity.twist.linear.y = msg->linear.y;
    velocity.twist.linear.z = msg->linear.z;
    velocity.twist.angular.z = msg->angular.z;
   
    velocity_publisher_.publish(velocity);

  }


  void cmdPoseCallback(const geometry_msgs::PoseConstPtr &msg)
  {
    ros::Time now = ros::Time::now();
    double dt = 0.0;
    if (!pose_.header.stamp.isZero()) {
      dt = std::max(0.0, std::min(1.0, (now - pose_.header.stamp).toSec()));
    }

  
    pose_.header.stamp = now;
    pose_.header.frame_id = world_frame_;
    pose_.pose.position = msg->position;  
    pose_.pose.orientation= msg->orientation; 
    // tf2::Quaternion q;
    // q.setRPY(0.0, 0.0, yaw_);
    // pose_.pose.orientation = tf2::toMsg(q);

    hector_uav_msgs::PoseGoal goal;
    goal.target_pose = pose_;
    pose_client_->sendGoal(goal);
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
