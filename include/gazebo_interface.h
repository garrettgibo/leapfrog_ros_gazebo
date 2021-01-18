#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math.hh>

using namespace gazebo;

class GazeboInterface : public ModelPlugin {
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);

private:
  std::unique_ptr<ros::NodeHandle> rosNode;
  physics::ModelPtr model;

  physics::LinkPtr link_thruster_1_;
  physics::LinkPtr link_thruster_2_;
  physics::LinkPtr link_thruster_3_;
  physics::LinkPtr link_thruster_4_;
  physics::LinkPtr link_actuator_1_;
  physics::LinkPtr link_actuator_2_;
  physics::LinkPtr link_engine_;

  std::string link_name_thruster_1_;
  std::string link_name_thruster_2_;
  std::string link_name_thruster_3_;
  std::string link_name_thruster_4_;
  std::string link_name_actuator_1_;
  std::string link_name_actuator_2_;
  std::string link_name_engine_;

  /// \brief ROS ACS Subscribers
  ros::Subscriber rosSubThrusters;

  /// \brief ROS Actuator Subscribers
  ros::Subscriber rosSubActuators;

  /// \brief ROS Engine Subscribers
  ros::Subscriber rosSubEngine;

  void ThrustersCallback(const std_msgs::Float32ConstPtr &_msg);
  void ActuatorsCallback(const std_msgs::Float32ConstPtr &_msg);
  void EngineCallback(const std_msgs::Float32ConstPtr &_msg);
};
