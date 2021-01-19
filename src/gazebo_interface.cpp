#include "gazebo_interface.h"

using namespace gazebo;

static const int NUM_THRUSTERS = 4;
static const int NUM_ACTUATORS = 2;

// Register plugin with gazebo 
GZ_REGISTER_MODEL_PLUGIN(GazeboInterface);

void GazeboInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  // Store pointer to the model
  this->model_ = _model;

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
  this->link_name_thrusters_[0] = _sdf->Get<std::string>("thruster_1");
  this->link_name_thrusters_[1] = _sdf->Get<std::string>("thruster_2");
  this->link_name_thrusters_[2] = _sdf->Get<std::string>("thruster_3");
  this->link_name_thrusters_[3] = _sdf->Get<std::string>("thruster_4");
  this->link_name_actuators_[0] = _sdf->Get<std::string>("actuator_1");
  this->link_name_actuators_[1] = _sdf->Get<std::string>("actuator_2");
  this->link_name_engine_ = _sdf->Get<std::string>("engine");

  // Get Links
  this->link_thrusters_[0] = _model->GetLink(link_name_thrusters_[0]);
  this->link_thrusters_[1] = _model->GetLink(link_name_thrusters_[1]);
  this->link_thrusters_[2] = _model->GetLink(link_name_thrusters_[2]);
  this->link_thrusters_[3] = _model->GetLink(link_name_thrusters_[3]);
  this->link_actuators_[0] = _model->GetLink(link_name_thrusters_[0]);
  this->link_actuators_[1] = _model->GetLink(link_name_thrusters_[1]);
  this->link_engine_ = _model->GetLink(link_name_engine_);

  ROS_INFO("Gazebo Interface Created");
};

void GazeboInterface::ThrustersCallback(const std_msgs::Float32MultiArray::ConstPtr &_msg) {
  for(int i=0; i<NUM_THRUSTERS; i++) {
    const ignition::math::Vector3<double>& force = { 0, 0, _msg->data[i]};
    this->link_thrusters_[i]->SetForce(force);
  };
}

void GazeboInterface::ActuatorsCallback(const std_msgs::Float32MultiArray::ConstPtr &_msg) {
  for(int i=0; i<NUM_ACTUATORS; i++) {
    // -x axis moves actuator outwards
    const ignition::math::Vector3<double>& force = {-1*_msg->data[i], 0, 0};
    this->link_actuators_[i]->SetForce(force);
  };
}

void GazeboInterface::EngineCallback(const std_msgs::Float32ConstPtr &_msg) {
  const ignition::math::Vector3<double>& force = { 0, 0, _msg->data};
  this->link_engine_->SetForce(force);
}
