#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Vector3.h"

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class BulletPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: BulletPlugin() {}
    public: ~BulletPlugin() {
      this->Reset();
    }

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Store the model pointer for convenience.
        this->model = _model;
        // Default to zero velocity
        double vel_x = 0;
        double vel_y = 0;
        double vel_z = 0;

        // Check that the velocity element exists, then read the value
        if (_sdf->HasElement("vx"))
            vel_x = _sdf->Get<double>("vx");
        if (_sdf->HasElement("vy"))
            vel_y = _sdf->Get<double>("vy");
        if (_sdf->HasElement("vz"))
            vel_z = _sdf->Get<double>("vz");

        // Set the joint's target velocity. This target velocity is just
        // for demonstration purposes.
        this->model->SetLinearVel(ignition::math::Vector3d(vel_x, vel_y, vel_z));

        // Create the node
        this->node = transport::NodePtr(new transport::Node());
        #if GAZEBO_MAJOR_VERSION < 8
            this->node->Init(this->model->GetWorld()->GetName());
        #else
            this->node->Init(this->model->GetWorld()->Name());
        #endif

        // Create a topic name
        std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

        // Subscribe to the topic, and register a callback
        this->sub = this->node->Subscribe(topicName,
           &BulletPlugin::OnMsg, this);

        // Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized())
        {
          int argc = 0;
          char **argv = NULL;
          ros::init(argc, argv, "gazebo_client",
              ros::init_options::NoSigintHandler);
        }

        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        // Create a named topic, and subscribe to it.
        ros::SubscribeOptions so =
          ros::SubscribeOptions::create<geometry_msgs::Vector3>(
              "/" + this->model->GetName() + "/vel_cmd",
              1,
              boost::bind(&BulletPlugin::OnRosMsg, this, _1),
              ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);

        // Spin up the queue helper thread.
        this->rosQueueThread =
          std::thread(std::bind(&BulletPlugin::QueueThread, this));
    }

    public: void SetVelocity(const double &_vel_x, const double &_vel_y, const double &_vel_z) {
          // Set the joint's target velocity.
          this->model->SetLinearVel(ignition::math::Vector3d(_vel_x, _vel_y, _vel_z));
      }

    private: void OnMsg(ConstVector3dPtr &_msg)
    {

      this->SetVelocity(_msg->x(), _msg->y(), _msg->z());
    }

    public: void OnRosMsg(const geometry_msgs::Vector3ConstPtr &_msg)
    {
      this->SetVelocity(_msg->x, _msg->y, _msg->z);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(BulletPlugin);
};
#endif
