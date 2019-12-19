//
// Created by shivesh on 12/16/19.
//

#ifndef DYNAMIC_OBJECTS_FUSION_SRC_FUSED_OBJECT_HPP_
#define DYNAMIC_OBJECTS_FUSION_SRC_FUSED_OBJECT_HPP_

#include <memory>
#include <unordered_map>
#include <vector>

#include "dynamic_objects_fusion/kalman_fusion.hpp"
#include "dynamic_objects_fusion/sensor_object.hpp"

namespace dynamic_objects_fusion {
class FusedObject {
 public:
  FusedObject(std::shared_ptr<SensorObject> sensor_object, double timestamp, int sensor_id);

  ~FusedObject();

  void update(std::shared_ptr<SensorObject> sensor_object, double timestamp);

  void update(double timestamp);

  double length_, width_, height_;
  double position_[3];
  double position_covariance_[3];
  double velocity_[3];
  double velocity_covariance_[3];
  double acceleration_[3];
  double acceleration_covariance_[3];
  double orientation_;
  double timestamp_;

  std::unordered_map<int, int> sensor_object_id_;

 private:
  KalmanFusion kalman_fusion_;
};

class FusedObjects {
 public:
  FusedObjects();

  ~FusedObjects();

  std::vector<std::shared_ptr<FusedObject>> objects_;
};
}

#endif //DYNAMIC_OBJECTS_FUSION_SRC_FUSED_OBJECT_HPP_
