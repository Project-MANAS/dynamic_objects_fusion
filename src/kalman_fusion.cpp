//
// Created by shivesh on 12/18/19.
//

#include "dynamic_objects_fusion/kalman_fusion.hpp"

namespace dynamic_objects_fusion {

KalmanFusion::KalmanFusion(std::shared_ptr<SensorObject> sensor_object) {
  current_position_(0) = sensor_object->position_[0];
  current_position_(1) = sensor_object->position_[1];
  current_position_(2) = sensor_object->position_[2];
  current_velocity_(0) = sensor_object->velocity_[0];
  current_velocity_(1) = sensor_object->velocity_[1];
  current_velocity_(2) = sensor_object->velocity_[2];
  current_acceleration_(0) = sensor_object->acceleration_[0];
  current_acceleration_(1) = sensor_object->acceleration_[1];
  current_acceleration_(2) = sensor_object->acceleration_[2];
  predicted_state_ << current_position_(0), current_position_(1), current_velocity_(0), current_velocity_(1);
}

KalmanFusion::~KalmanFusion() {

}

void KalmanFusion::update(std::shared_ptr<SensorObject> sensor_object, double current_timestamp) {
  double delta_time = current_timestamp - timestamp_;

  state_transition_.setIdentity();
  state_transition_(0, 2) = delta_time;
  state_transition_(1, 3) = delta_time;
  current_state_ = state_transition_ * predicted_state_;
  current_state_(2) += current_acceleration_(0) * delta_time;
  current_state_(3) += current_acceleration_(1) * delta_time;

  Eigen::Vector4d measurement;
  measurement << sensor_object->position_[0], sensor_object->position_[1], sensor_object->velocity_[0], sensor_object->velocity_[1];

  kalman_gain_.setIdentity();
  // TODO Update Kalman gain

  predicted_state_ = current_state_ + kalman_gain_ * (measurement - current_state_);
  current_position_(0) = predicted_state_(0);
  current_position_(1) = predicted_state_(1);
  current_velocity_(0) = predicted_state_(2);
  current_velocity_(1) = predicted_state_(3);
  timestamp_ = current_timestamp;
}

void KalmanFusion::update(double current_timestamp) {
  double delta_time = current_timestamp - timestamp_;
  current_position_ += (current_velocity_ * delta_time);
  timestamp_ = current_timestamp;
}

void KalmanFusion::get_state(double position[3], double velocity[3], double& timestamp) {
  get_state(position, velocity);
  timestamp = timestamp_;
}

void KalmanFusion::get_state(double position[3], double velocity[3]) {
  position[0] = current_position_(0);
  position[1] = current_position_(1);
  position[2] = current_position_(2);
  velocity[0] = current_velocity_(0);
  velocity[1] = current_velocity_(1);
  velocity[2] = current_velocity_(2);
}

}