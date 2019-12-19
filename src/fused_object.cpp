//
// Created by shivesh on 12/16/19.
//

#include <iostream>
#include "dynamic_objects_fusion/fused_object.hpp"

namespace dynamic_objects_fusion {

FusedObject::FusedObject(std::shared_ptr<SensorObject> sensor_object, double timestamp, int sensor_id) :
  timestamp_(timestamp),
  kalman_fusion_(sensor_object) {
  sensor_object_id_[sensor_id] = sensor_object->id_;
  length_ = sensor_object->length_;
  width_ = sensor_object->width_;
//  height_ = sensor_object->height_;

  position_[0] = sensor_object->position_[0];
  position_[1] = sensor_object->position_[1];
  position_[2] = sensor_object->position_[2];

  position_covariance_[0] = sensor_object->position_covariance_[0];
  position_covariance_[1] = sensor_object->position_covariance_[1];
  position_covariance_[2] = sensor_object->position_covariance_[2];

  velocity_[0] = sensor_object->velocity_[0];
  velocity_[1] = sensor_object->velocity_[1];
  velocity_[2] = sensor_object->velocity_[2];

  velocity_covariance_[0] = sensor_object->velocity_covariance_[0];
  velocity_covariance_[1] = sensor_object->velocity_covariance_[1];
  velocity_covariance_[2] = sensor_object->velocity_covariance_[2];

  acceleration_[0] = sensor_object->acceleration_[0];
  acceleration_[1] = sensor_object->acceleration_[1];
  acceleration_[2] = sensor_object->acceleration_[2];

  acceleration_covariance_[0] = sensor_object->acceleration_covariance_[0];
  acceleration_covariance_[1] = sensor_object->acceleration_covariance_[1];
  acceleration_covariance_[2] = sensor_object->acceleration_covariance_[2];
  orientation_ = sensor_object->orientation_;
}
FusedObject::~FusedObject() {

}

void FusedObject::update(std::shared_ptr<SensorObject> sensor_object, double timestamp) {
  length_ = sensor_object->length_;
  width_ = sensor_object->width_;
  orientation_ = sensor_object->orientation_;
  kalman_fusion_.update(sensor_object, timestamp);
  kalman_fusion_.get_state(position_, velocity_, timestamp_);
}

void FusedObject::update(double timestamp) {
  kalman_fusion_.update(timestamp);
  kalman_fusion_.get_state(position_, velocity_);
}

FusedObjects::FusedObjects() {

}
FusedObjects::~FusedObjects() {

}
}