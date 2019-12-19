//
// Created by shivesh on 12/16/19.
//

#ifndef DYNAMIC_OBJECTS_FUSION_SRC_DYNAMIC_OBJECTS_FUSION_HPP_
#define DYNAMIC_OBJECTS_FUSION_SRC_DYNAMIC_OBJECTS_FUSION_HPP_

#include <unordered_map>
#include <unordered_set>

#include "dynamic_objects_fusion/fused_object.hpp"
#include "dynamic_objects_fusion/hungarian_bigraph_matcher.hpp"
#include "dynamic_objects_fusion/sensor_object.hpp"

namespace dynamic_objects_fusion {
class DynamicObjectsFusion {
 public:
  DynamicObjectsFusion();

  ~DynamicObjectsFusion();

  void fuse(std::shared_ptr<SensorObjects> sensor_objects);

  FusedObjects fused_objects_;

 private:
  std::shared_ptr<std::unordered_map<int, int>> associate_old_objects(
      std::shared_ptr<SensorObjects> sensor_objects,
      std::shared_ptr<std::vector<int>> unassociated_sensor_objects,
      std::shared_ptr<std::vector<int>> unassociated_fused_objects);

  std::shared_ptr<std::vector<std::vector<double>>> compute_association_matrix(
      std::shared_ptr<SensorObjects> sensor_objects,
      std::shared_ptr<std::vector<int>> unassociated_sensor_objects,
      std::shared_ptr<std::vector<int>> unassociated_fused_objects);

  void associate_new_objects(std::shared_ptr<SensorObjects> sensor_objects,
      std::shared_ptr<std::vector<std::vector<double>>> association_matrix,
      std::shared_ptr<std::unordered_map<int, int>> association,
      std::shared_ptr<std::vector<int>> unassociated_sensor_objects,
      std::shared_ptr<std::vector<int>> unassociated_fused_objects);

  double compute_distance(std::shared_ptr<FusedObject> fused_object, std::shared_ptr<SensorObject> sensor_object);

  void update_associated_fused_objects(std::shared_ptr<SensorObjects> sensor_objects, std::shared_ptr<std::unordered_map<int, int>> association);

  void update_unassociated_fused_objects(std::shared_ptr<std::vector<int>> unassociated_fused_objects, double timestamp);

  void create_new_fused_objects(std::shared_ptr<SensorObjects> sensor_objects,
      std::shared_ptr<std::vector<int>> unassociated_sensor_objects);

  void delete_expired_fused_objects(double timestamp);

  virtual void send_fused_objects();
};
}

#endif //DYNAMIC_OBJECTS_FUSION_SRC_DYNAMIC_OBJECTS_FUSION_HPP_
