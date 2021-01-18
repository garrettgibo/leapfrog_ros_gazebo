#include "gazebo_interface.h"

using namespace gazebo;

// Register plugin with gazebo 
GZ_REGISTER_MODEL_PLUGIN(GazeboInterface);

void GazeboInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  // Store pointer to the model
  this->model = _model;

  rosNode.reset(new ros::NodeHandle("gazebo_client"));

  // Create topics for thrusters, actuators, and engine
  rosSubThrusters =
    rosNode->subscribe("/" + _model->GetName() + "/thrusters", 1000,
		       &GazeboInterface::ThrustersCallback, this);
  rosSubActuators =
    rosNode->subscribe("/" + _model->GetName() + "/actuators", 1000,
		       &GazeboInterface::ActuatorsCallback, this);
  rosSubEngine =
    rosNode->subscribe("/" + _model->GetName() + "/engine", 1000,
		       &GazeboInterface::EngineCallback, this);

  // Get SDF parameters
  this->link_name_thruster_1_ = _sdf->Get<std::string>("thruster_1");
  this->link_name_thruster_2_ = _sdf->Get<std::string>("thruster_2");
  this->link_name_thruster_3_ = _sdf->Get<std::string>("thruster_3");
  this->link_name_thruster_4_ = _sdf->Get<std::string>("thruster_4");
  this->link_name_actuator_1_ = _sdf->Get<std::string>("actuator_1");
  this->link_name_actuator_2_ = _sdf->Get<std::string>("actuator_2");
  this->link_name_engine_ = _sdf->Get<std::string>("engine");
  ROS_INFO_STREAM(this->link_name_engine_);

  // Get Links
  this->link_thruster_1_ = _model->GetLink(link_name_thruster_1_);
  this->link_thruster_2_ = _model->GetLink(link_name_thruster_2_);
  this->link_thruster_3_ = _model->GetLink(link_name_thruster_3_);
  this->link_thruster_4_ = _model->GetLink(link_name_thruster_4_);
  this->link_actuator_1_ = _model->GetLink(link_name_actuator_1_);
  this->link_actuator_2_ = _model->GetLink(link_name_actuator_2_);
  this->link_engine_ = _model->GetLink(link_name_engine_);

  ROS_INFO("Gazebo Interface Created");
};

void GazeboInterface::ThrustersCallback(const std_msgs::Float32ConstPtr &_msg) {
  ROS_INFO("Thruster Value: %f", _msg->data);
}

void GazeboInterface::ActuatorsCallback(const std_msgs::Float32ConstPtr &_msg) {
  ROS_INFO("Actuators Value: %f", _msg->data);
}

void GazeboInterface::EngineCallback(const std_msgs::Float32ConstPtr &_msg) {
  const ignition::math::Vector3<double>& force = { 0, 0, _msg->data};
  ROS_INFO("Engine Value: %f", _msg->data);
  this->link_engine_->SetForce(force);
}

