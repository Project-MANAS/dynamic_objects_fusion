//
// Created by shivesh on 12/16/19.
//

#ifndef DYNAMIC_OBJECTS_FUSION_SENSOR_OBJECT_HPP
#define DYNAMIC_OBJECTS_FUSION_SENSOR_OBJECT_HPP

#include <vector>
#include <memory>

namespace dynamic_objects_fusion {
class SensorObject {
 public:
  SensorObject();

  ~SensorObject();

  explicit SensorObject(std::shared_ptr<SensorObject> sensor_object);

  int id_;
  double length_, width_, height_;
  double position_[3];
  double position_covariance_[3];
  double velocity_[3];
  double velocity_covariance_[3];
  double acceleration_[3];
  double acceleration_covariance_[3];
  double orientation_;
};

class SensorObjects {
 public:
  SensorObjects();

  SensorObjects(int id, double timestamp);

  ~SensorObjects();

  std::vector<std::shared_ptr<SensorObject>> objects_;

  int sensor_id_;
  double timestamp_;
};
}

#endif //DYNAMIC_OBJECTS_FUSION_SENSOR_OBJECT_HPP
