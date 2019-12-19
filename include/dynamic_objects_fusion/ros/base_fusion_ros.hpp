//
// Created by shivesh on 12/14/19.
//

#ifndef DYNAMIC_OBJECTS_FUSION_BASE_FUSION_ROS_HPP
#define DYNAMIC_OBJECTS_FUSION_BASE_FUSION_ROS_HPP

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <dynamic_objects_fusion/ObjectsList.h>
#include "dynamic_objects_fusion/dynamic_objects_fusion.hpp"

namespace dynamic_objects_fusion {
class BaseFusionROS {
 public:
  BaseFusionROS();

  ~BaseFusionROS();

  void initialize(DynamicObjectsFusion* dynamic_objects_fusion, int id);

  std::shared_ptr<SensorObject> object_to_sensor_object(const Object object);

  DynamicObjectsFusion* dynamic_objects_fusion_;
  int id_;
};
}

#endif //DYNAMIC_OBJECTS_FUSION_BASE_FUSION_ROS_HPP
