//
// Created by shivesh on 12/14/19.
//

#ifndef DYNAMIC_OBJECTS_TRACKER_DYNAMIC_OBJECTS_TRACKER_ROS_HPP
#define DYNAMIC_OBJECTS_TRACKER_DYNAMIC_OBJECTS_TRACKER_ROS_HPP

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include "dynamic_objects_fusion/dynamic_objects_fusion.hpp"
#include "dynamic_objects_fusion/ros/base_fusion_ros.hpp"
#include "dynamic_objects_fusion/ros/radar_fusion_ros.hpp"

namespace dynamic_objects_fusion {
class DynamicObjectsFusionROS : public DynamicObjectsFusion {
 public:
  explicit DynamicObjectsFusionROS(ros::NodeHandle nh);

  void send_fused_objects() override;

 private:
  void fused_object_to_object(std::shared_ptr<FusedObject> fused_object, Object &object);

  pluginlib::ClassLoader<BaseFusionROS> plugin_loader_;

  ros::Publisher fused_objects_pub_;

  RadarFusionROS radar_fusion_ros_;
};
}

#endif //DYNAMIC_OBJECTS_TRACKER_DYNAMIC_OBJECTS_TRACKER_ROS_HPP
