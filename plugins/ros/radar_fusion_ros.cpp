//
// Created by shivesh on 12/14/19.
//

#include "dynamic_objects_fusion/ros/radar_fusion_ros.hpp"

namespace dynamic_objects_fusion {
RadarFusionROS::RadarFusionROS() {

}

RadarFusionROS::RadarFusionROS(ros::NodeHandle nh) :
  radar_objects_topic_("/ars_40X/objects")
{
  radar_objects_sub_ = nh.subscribe(radar_objects_topic_, 1, &RadarFusionROS::radar_objects_callback, this);
}

RadarFusionROS::~RadarFusionROS() {

}

void RadarFusionROS::radar_objects_callback(const dynamic_objects_fusion::ObjectsList objects_list) {
  auto sensor_objects = std::make_shared<SensorObjects>(id_, objects_list.header.stamp.toSec());
  for (const auto & object : objects_list.objects) {
    sensor_objects->objects_.push_back(object_to_sensor_object(object));
  }
  dynamic_objects_fusion_->fuse(sensor_objects);
}
}