//
// Created by shivesh on 12/17/19.
//

#ifndef DYNAMIC_OBJECTS_FUSION_SRC_ROS_DYNAMIC_OBJECTS_FUSION_RVIZ_HPP_
#define DYNAMIC_OBJECTS_FUSION_SRC_ROS_DYNAMIC_OBJECTS_FUSION_RVIZ_HPP_

#include <ros/ros.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>

#include "dynamic_objects_fusion/ObjectsList.h"

namespace dynamic_objects_fusion {
class DynamicObjectsFusionRViz {
 public:
  explicit DynamicObjectsFusionRViz(ros::NodeHandle nh);

  ~DynamicObjectsFusionRViz();

 private:
  void objects_callback(ObjectsList objects_list);

  ros::Publisher objects_pub_;

  ros::Publisher velocity_pub_;

  ros::Subscriber fused_objects_sub_;
};
}

#endif //DYNAMIC_OBJECTS_FUSION_SRC_ROS_DYNAMIC_OBJECTS_FUSION_RVIZ_HPP_
