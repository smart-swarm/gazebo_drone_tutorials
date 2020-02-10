


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
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
  ros::Subscriber pose_cmd_subscriber_;
  // ros::Publisher pose_publisher_;
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
    ros::NodeHandle robot_nh;

    // TODO factor out
    robot_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    robot_nh.param<std::string>("world_frame", world_frame_, "world");
    robot_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");

    pose_cmd_subscriber_ = node_handle_.subscribe<geometry_msgs::Pose>("pose_cmd", 1, boost::bind(&Teleop::PoseCmdTwistCallback, this, _1));
    // pose_publisher_ = robot_nh.advertise<geometry_msgs::PoseStamped>("command/pose",10);

    pose_.pose.position.x = 0;
    pose_.pose.position.y = 0;
    pose_.pose.position.z = 0;
    pose_.pose.orientation.x = 0;
    pose_.pose.orientation.y = 0;
    pose_.pose.orientation.z = 0;
    pose_.pose.orientation.w = 1;
  
  

    motor_enable_service_ = robot_nh.serviceClient<hector_uav_msgs::EnableMotors>(
        "enable_motors");
    takeoff_client_ = boost::shared_ptr<TakeoffClient>(new TakeoffClient(robot_nh, "action/takeoff"));
    landing_client_ = boost::shared_ptr<LandingClient>(new LandingClient(robot_nh, "action/landing"));
    pose_client_ = boost::shared_ptr<PoseClient>(new PoseClient(robot_nh, "action/pose"));
  }

  ~Teleop()
  {
    // stop();
  }

  void PoseCmdTwistCallback(const geometry_msgs::PoseConstPtr &pos_msg)
  {
      pose_.header.frame_id=base_stabilized_frame_;
      pose_.header.stamp = ros::Time::now();
      pose_.pose.position.x = pos_msg->position.x;
      pose_.pose.position.y = pos_msg->position.y;
      pose_.pose.position.z = pos_msg->position.z;
      yaw_ = pos_msg->orientation.z;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw_);
      pose_.pose.orientation = tf2::toMsg(q);

      // pose_publisher_.publish(pose_);

      hector_uav_msgs::PoseGoal goal;
      goal.target_pose = pose_;
      std::cout<<pose_<<std::endl;
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

//   void stop()
//   {
//     if (velocity_publisher_.getNumSubscribers() > 0)
//     {
//       velocity_publisher_.publish(geometry_msgs::TwistStamped());
//     }
//     if (attitude_publisher_.getNumSubscribers() > 0)
//     {
//       attitude_publisher_.publish(hector_uav_msgs::AttitudeCommand());
//     }
//     if (thrust_publisher_.getNumSubscribers() > 0)
//     {
//       thrust_publisher_.publish(hector_uav_msgs::ThrustCommand());
//     }
//     if (yawrate_publisher_.getNumSubscribers() > 0)
//     {
//       yawrate_publisher_.publish(hector_uav_msgs::YawrateCommand());
//     }
//   }
};

} // namespace hector_quadrotor

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quadrotor_teleop");

  hector_quadrotor::Teleop teleop;
  ros::spin();

  return 0;
}
