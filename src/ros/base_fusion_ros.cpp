//
// Created by shivesh on 12/16/19.
//

#include "dynamic_objects_fusion/ros/base_fusion_ros.hpp"

namespace dynamic_objects_fusion {

BaseFusionROS::BaseFusionROS() {

}
BaseFusionROS::~BaseFusionROS() {

}
void BaseFusionROS::initialize(DynamicObjectsFusion* dynamic_objects_fusion, int id) {
  dynamic_objects_fusion_ = dynamic_objects_fusion;
  id_ = id;
}

std::shared_ptr<SensorObject> BaseFusionROS::object_to_sensor_object(const dynamic_objects_fusion::Object object) {
  std::shared_ptr<SensorObject> sensor_object = std::make_shared<SensorObject>();
  sensor_object->id_ = object.id;
  sensor_object->length_ = object.length;
  sensor_object->width_ = object.width;
//  sensor_object->height_ = object.height;
  tf2::Quaternion q;
  q.setValue(
      object.position.pose.orientation.x,
      object.position.pose.orientation.y,
      object.position.pose.orientation.z,
      object.position.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  if (isnan(yaw)) {
    yaw = 0;
  }
  sensor_object->orientation_ = yaw;
  sensor_object->position_[0] = object.position.pose.position.x;
  sensor_object->position_[1] = object.position.pose.position.y;
  sensor_object->position_[2] = object.position.pose.position.z;
  sensor_object->position_covariance_[0] = object.position.covariance[0];
  sensor_object->position_covariance_[1] = object.position.covariance[7];
  sensor_object->position_covariance_[2] = object.position.covariance[14];
  sensor_object->velocity_[0] = object.relative_velocity.twist.linear.x;
  sensor_object->velocity_[1] = object.relative_velocity.twist.linear.y;
  sensor_object->velocity_[2] = object.relative_velocity.twist.linear.z;
  sensor_object->velocity_covariance_[0] = object.relative_velocity.covariance[0];
  sensor_object->velocity_covariance_[1] = object.relative_velocity.covariance[7];
  sensor_object->velocity_covariance_[2] = object.relative_velocity.covariance[14];
  sensor_object->acceleration_[0] = object.relative_acceleration.accel.linear.x;
  sensor_object->acceleration_[1] = object.relative_acceleration.accel.linear.y;
  sensor_object->acceleration_[2] = object.relative_acceleration.accel.linear.z;
  sensor_object->acceleration_covariance_[0] = object.relative_acceleration.covariance[0];
  sensor_object->acceleration_covariance_[1] = object.relative_acceleration.covariance[7];
  sensor_object->acceleration_covariance_[2] = object.relative_acceleration.covariance[14];
  return sensor_object;
}


}