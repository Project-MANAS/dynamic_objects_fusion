//
// Created by shivesh on 12/14/19.
//

#include "dynamic_objects_fusion/ros/dynamic_objects_fusion_ros.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_objects_fusion");
  ros::NodeHandle nh;
  dynamic_objects_fusion::DynamicObjectsFusionROS dynamic_objects_fusion_ros(nh);
  ros::spin();
}