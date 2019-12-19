//
// Created by shivesh on 12/14/19.
//

#ifndef DYNAMIC_OBJECTS_FUSION_RADAR_FUSION_ROS_HPP
#define DYNAMIC_OBJECTS_FUSION_RADAR_FUSION_ROS_HPP

#include "dynamic_objects_fusion/ros/base_fusion_ros.hpp"

namespace dynamic_objects_fusion {
class RadarFusionROS : public BaseFusionROS {
 public:
  RadarFusionROS();
  RadarFusionROS(ros::NodeHandle nh);

  ~RadarFusionROS();

 private:
  void radar_objects_callback(const dynamic_objects_fusion::ObjectsList objects_list);

  std::string radar_objects_topic_;

  ros::Subscriber radar_objects_sub_;
};
}

#endif //DYNAMIC_OBJECTS_FUSION_RADAR_FUSION_ROS_HPP
