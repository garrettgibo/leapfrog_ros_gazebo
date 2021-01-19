#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math.hh>

using namespace gazebo;

class GazeboInterface : public ModelPlugin {
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);

private:
  std::unique_ptr<ros::NodeHandle> rosNode;
  physics::ModelPtr model_;

  // Array of thrusters and actuators
  physics::LinkPtr link_thrusters_[4];
  physics::LinkPtr link_actuators_[2];
  physics::LinkPtr link_engine_;

  // Array of thruster and actuator link names;
  std::string link_name_thrusters_[4];
  std::string link_name_actuators_[2];
  std::string link_name_engine_;



  /// \brief ROS ACS Subscribers
  ros::Subscriber rosSubThrusters;

  /// \brief ROS Actuator Subscribers
  ros::Subscriber rosSubActuators;

  /// \brief ROS Engine Subscribers
  ros::Subscriber rosSubEngine;

  void ThrustersCallback(const std_msgs::Float32MultiArray::ConstPtr &_msg);
  void ActuatorsCallback(const std_msgs::Float32MultiArray::ConstPtr &_msg);
  void EngineCallback(const std_msgs::Float32ConstPtr &_msg);
};
