//
// Created by shivesh on 12/18/19.
//

#ifndef DYNAMIC_OBJECTS_FUSION_SRC_KALMAN_FUSION_HPP_
#define DYNAMIC_OBJECTS_FUSION_SRC_KALMAN_FUSION_HPP_

#include <eigen3/Eigen/Core>
#include "dynamic_objects_fusion/sensor_object.hpp"

namespace dynamic_objects_fusion {
class KalmanFusion {
 public:
  KalmanFusion(std::shared_ptr<SensorObject> sensor_object);

  ~KalmanFusion();

  void update(std::shared_ptr<SensorObject> sensor_object, double current_timestamp);

  void update(double current_timestamp);

  void get_state(double position[3], double velocity[3], double& timestamp);

  void get_state(double position[3], double velocity[3]);

 private:
  Eigen::Vector3d current_position_;

  Eigen::Vector3d current_velocity_;

  Eigen::Vector3d current_acceleration_;

  Eigen::Matrix4d state_transition_;

  Eigen::Matrix4d kalman_gain_;

  Eigen::Vector4d current_state_;

  Eigen::Vector4d predicted_state_;

  double timestamp_;
};
}

#endif //DYNAMIC_OBJECTS_FUSION_SRC_KALMAN_FUSION_HPP_
