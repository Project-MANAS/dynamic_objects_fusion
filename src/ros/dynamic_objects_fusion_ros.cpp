//
// Created by shivesh on 12/14/19.
//

#include "dynamic_objects_fusion/ros/dynamic_objects_fusion_ros.hpp"

namespace dynamic_objects_fusion {
DynamicObjectsFusionROS::DynamicObjectsFusionROS(ros::NodeHandle nh) :
  radar_fusion_ros_(nh),
  plugin_loader_("dynamic_objects_fusion", "dynamic_objects_fusion::BaseFusionROS")
{
  radar_fusion_ros_.initialize(this, 0);
//  boost::shared_ptr<BaseFusionROS> plugin = plugin_loader_.createInstance("dynamic_objects_fusion::RadarFusionROS");
//  plugin->initialize(this, 0);
//
//  if (nh.hasParam("plugins")) {
//    XmlRpc::XmlRpcValue plugins_list;
//    nh.getParam("plugins", plugins_list);
//    for (int i = 0; i < plugins_list.size(); ++i) {
//      std::string pname = static_cast<std::string>(plugins_list[i]["name"]);
//      std::string type = static_cast<std::string>(plugins_list[i]["type"]);
//      ROS_INFO("Using plugin \"%s\"", pname.c_str());
//
//      boost::shared_ptr<BaseFusionROS> plugin = plugin_loader_.createInstance(type);
//      plugin->initialize(this, i);
//    }
//  }

  fused_objects_pub_ = nh.advertise<dynamic_objects_fusion::ObjectsList>("/fused_objects", 10);
}

void DynamicObjectsFusionROS::send_fused_objects() {
  ObjectsList objects_list;
  objects_list.header.stamp = ros::Time::now();
  for (int id = 0; id < fused_objects_.objects_.size(); ++id) {
    Object object;
    object.id = id;
    fused_object_to_object(fused_objects_.objects_[id], object);
    objects_list.objects.push_back(object);
  }
  fused_objects_pub_.publish(objects_list);
}

void DynamicObjectsFusionROS::fused_object_to_object(const std::shared_ptr<FusedObject> fused_object, Object &object) {
  object.length = fused_object->length_;
  object.width = fused_object->width_;
//  object.height = fused_object->height_;
  object.orientation_angle = fused_object->orientation_;

  tf2::Quaternion q;
  q.setRPY(0, 0, object.orientation_angle);
  object.position.pose.orientation.w = q.getW();
  object.position.pose.orientation.x = q.getX();
  object.position.pose.orientation.y = q.getY();
  object.position.pose.orientation.z = q.getZ();

  object.position.pose.position.x = fused_object->position_[0];
  object.position.pose.position.y = fused_object->position_[1];
  object.position.pose.position.z = fused_object->position_[2];
  object.position.covariance[0] = fused_object->position_covariance_[0];
  object.position.covariance[7] = fused_object->position_covariance_[1];
  object.position.covariance[14] = fused_object->position_covariance_[2];
  object.relative_velocity.twist.linear.x = fused_object->velocity_[0];
  object.relative_velocity.twist.linear.y = fused_object->velocity_[1];
  object.relative_velocity.twist.linear.z = fused_object->velocity_[2];
  object.relative_velocity.covariance[0] = fused_object->velocity_covariance_[0];
  object.relative_velocity.covariance[7] = fused_object->velocity_covariance_[1];
  object.relative_velocity.covariance[14] = fused_object->velocity_covariance_[2];
  object.relative_acceleration.accel.linear.x = fused_object->acceleration_[0];
  object.relative_acceleration.accel.linear.y = fused_object->acceleration_[1];
  object.relative_acceleration.accel.linear.z = fused_object->acceleration_[2];
  object.relative_acceleration.covariance[0] = fused_object->acceleration_covariance_[0];
  object.relative_acceleration.covariance[7] = fused_object->acceleration_covariance_[1];
  object.relative_acceleration.covariance[14] = fused_object->acceleration_covariance_[2];
}

}