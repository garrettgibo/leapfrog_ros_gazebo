#ifndef _CONTROL_LINKS_PLUGIN_HH_
#define _CONTROL_LINKS_PLUGIN_HH_

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
// #include <thread>

namespace gazebo {
  /// \brief A plugin to control a Velodyne sensor.
  class ControlLinksPlugin : public ModelPlugin {
    /// \brief A node use for ROS transport
  private:
    std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS ACS Subscribers
  private:
    ros::Subscriber rosSubACS1;

  private:
    ros::Subscriber rosSubACS2;

  private:
    ros::Subscriber rosSubACS3;

  private:
    ros::Subscriber rosSubACS4;

    /// \brief ROS Actuator Subscribers
  private:
    ros::Subscriber rosSubActuator1;

  private:
    ros::Subscriber rosSubActuator2;

    /// \brief ROS Engine Subscribers
  private:
    ros::Subscriber rosSubEngine;

    /// \brief A ROS callbackqueue that helps process messages
  // private:
  //   ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
  // private:
  //   std::thread rosQueueThread;

    /// \brief Constructor
  public:
    ControlLinksPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
  public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized()) {
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "gazebo_client",
		  ros::init_options::NoSigintHandler);
      }
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      this->rosSubACS1 =
	this->rosNode->subscribe("/" + _model->GetName() + "/acs_1", 1000,
				 &ControlLinksPlugin::OnRosMsg, this);
      this->rosSubACS2 =
	this->rosNode->subscribe("/" + _model->GetName() + "/acs_2", 1000,
				 &ControlLinksPlugin::OnRosMsg, this);
      this->rosSubACS3 =
	this->rosNode->subscribe("/" + _model->GetName() + "/acs_3", 1000,
				 &ControlLinksPlugin::OnRosMsg, this);
      this->rosSubACS4 =
	this->rosNode->subscribe("/" + _model->GetName() + "/acs_4", 1000,
				 &ControlLinksPlugin::OnRosMsg, this);
      this->rosSubActuator1 =
	this->rosNode->subscribe("/" + _model->GetName() + "/actuator_1", 1000,
				 &ControlLinksPlugin::OnRosMsg, this);
      this->rosSubActuator2 =
	this->rosNode->subscribe("/" + _model->GetName() + "/actuator_1", 1000,
				 &ControlLinksPlugin::OnRosMsg, this);
      this->rosSubEngine =
	this->rosNode->subscribe("/" + _model->GetName() + "/engine", 1000,
				 &ControlLinksPlugin::OnRosMsg, this);

      // Spin up the queue helper thread.
      // this->rosQueueThread =
      // 	std::thread(std::bind(&ControlLinksPlugin::QueueThread, this));

      ROS_INFO_STREAM("Control Links Plugin is attached to model ["
		      << _model->GetName() << "]");
    }
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
  public:
    void OnRosMsg(const std_msgs::Float32ConstPtr &_msg) {
      ROS_INFO("%f", _msg->data);
    }

    //   /// \brief ROS helper function that processes messages
    // private:
    //   void QueueThread() {
    //     static const double timeout = 0.01;
    //     while (this->rosNode->ok()) {
    //       this->rosQueue.callAvailable(ros::WallDuration(timeout));
    //     }
    //   }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(ControlLinksPlugin)
} // namespace gazebo
#endif
