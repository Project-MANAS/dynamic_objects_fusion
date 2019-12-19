//
// Created by shivesh on 12/17/19.
//

#include "dynamic_objects_fusion/ros/dynamic_objects_fusion_rviz.hpp"

namespace dynamic_objects_fusion {

DynamicObjectsFusionRViz::DynamicObjectsFusionRViz(ros::NodeHandle nh) {
  objects_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_objects2", 10);
  velocity_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_velocity2", 10);

  fused_objects_sub_ = nh.subscribe(
      "/fused_objects", 10, &DynamicObjectsFusionRViz::objects_callback, this);
}
DynamicObjectsFusionRViz::~DynamicObjectsFusionRViz() {

}

void DynamicObjectsFusionRViz::objects_callback(ObjectsList objects_list) {

  visualization_msgs::MarkerArray marker_array, velocity_array;
  for (auto object : objects_list.objects) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.header.frame_id = "front_center_laser_link";
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = objects_list.header.stamp;
    marker.id = object.id;
    geometry_msgs::Point pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
    tf2::Quaternion q;
    q.setValue(
        object.position.pose.orientation.x,
        object.position.pose.orientation.y,
        object.position.pose.orientation.z,
        object.position.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    if (isnan(yaw)) {
      continue;
    }
    marker.pose = object.position.pose;
    pos1.x = object.length / 2;
    pos1.y = object.width / 2;
//    pos1.z = object.height / 2;
    pos2.x = object.length / 2;
    pos2.y = -object.width / 2;
//    pos2.z = object.height / 2;
    pos3.x = -object.length / 2;
    pos3.y = -object.width / 2;
//    pos3.z = object.height / 2;
    pos4.x = -object.length / 2;
    pos4.y = object.width / 2;
//    pos4.z = object.height / 2;
    pos5.x = object.length / 2;
    pos5.y = object.width / 2;
//    pos5.z = -object.height / 2;
    pos6.x = object.length / 2;
    pos6.y = -object.width / 2;
//    pos6.z = -object.height / 2;
    pos7.x = -object.length / 2;
    pos7.y = -object.width / 2;
//    pos7.z = -object.height / 2;
    pos8.x = -object.length / 2;
    pos8.y = object.width / 2;
//    pos8.z = -object.height / 2;
    marker.points.push_back(pos1);
    marker.points.push_back(pos2);
    marker.points.push_back(pos3);
    marker.points.push_back(pos4);
    marker.points.push_back(pos1);
    marker.points.push_back(pos5);
    marker.points.push_back(pos6);
    marker.points.push_back(pos7);
    marker.points.push_back(pos8);
    marker.points.push_back(pos5);
    marker.scale.x = 0.1;

    marker.color.a = 1.0;
    marker.lifetime.fromSec(0.1);
    marker_array.markers.push_back(marker);

    visualization_msgs::Marker velocity;
    velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    velocity.header.frame_id = objects_list.header.frame_id;
    velocity.action = visualization_msgs::Marker::ADD;
    velocity.header.stamp = objects_list.header.stamp;
    velocity.id = object.id;
    velocity.text = std::to_string(hypot(object.relative_velocity.twist.linear.x, object.relative_velocity.twist.linear.y));
    velocity.pose = object.position.pose;
    velocity.scale.z = 1;
    velocity.color.a = 1;

    velocity_array.markers.push_back(velocity);
  }
  objects_pub_.publish(marker_array);
//  velocity_pub_.publish(velocity_array);
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_objects_fusion_rviz");
  ros::NodeHandle nh;
  dynamic_objects_fusion::DynamicObjectsFusionRViz dynamic_objects_fusion_rviz(nh);
  ros::spin();
}