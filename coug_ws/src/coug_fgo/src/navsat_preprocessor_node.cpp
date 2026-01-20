// Copyright (c) 2026 BYU FRoSt Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file navsat_preprocessor_node.cpp
 * @brief Implementation of the NavsatPreprocessorNode.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#include "coug_fgo/navsat_preprocessor_node.hpp"

#include <geodesy/wgs84.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geographic_msgs/msg/geo_point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace coug_fgo
{

NavsatPreprocessorNode::NavsatPreprocessorNode()
: Node("navsat_preprocessor_node")
{
  RCLCPP_INFO(get_logger(), "Starting NavSat Preprocessor Node...");

  // --- Parameters ---
  double origin_pub_rate = declare_parameter<double>("origin_pub_rate", 1.0);
  set_origin_ = declare_parameter<bool>("set_origin", true);
  std::string input_topic = declare_parameter<std::string>("input_topic", "gps/fix");
  std::string odom_output_topic =
    declare_parameter<std::string>("odom_output_topic", "odometry/gps");
  std::string origin_topic =
    declare_parameter<std::string>("origin_topic", "/origin");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");

  bool use_parameter_origin = declare_parameter<bool>("use_parameter_origin", false);
  double parameter_origin_lat = declare_parameter<double>("parameter_origin.latitude", 40.33940);
  double parameter_origin_lon =
    declare_parameter<double>("parameter_origin.longitude", -111.90721);
  double parameter_origin_alt = declare_parameter<double>("parameter_origin.altitude", 1412.0);

  // --- ROS Interfaces ---
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_output_topic, 10);

  if (set_origin_) {
    origin_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(origin_topic, 10);
  } else {
    origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      origin_topic, 10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {originCallback(msg);});
  }

  if (use_parameter_origin && set_origin_) {
    try {
      geographic_msgs::msg::GeoPoint pt;
      pt.latitude = parameter_origin_lat;
      pt.longitude = parameter_origin_lon;
      pt.altitude = parameter_origin_alt;
      origin_utm_ = geodesy::UTMPoint(pt);

      origin_navsat_.header.frame_id = map_frame_;
      origin_navsat_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      origin_navsat_.latitude = parameter_origin_lat;
      origin_navsat_.longitude = parameter_origin_lon;
      origin_navsat_.altitude = parameter_origin_alt;

      origin_set_ = true;

      RCLCPP_INFO(
        get_logger(), "Parameter Origin Set: Lat %.6f, Lon %.6f (UTM Zone %d%c)",
        origin_navsat_.latitude, origin_navsat_.longitude, origin_utm_.zone,
        origin_utm_.band);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Parameter origin set failed: %s", e.what());
    }
  }

  navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    input_topic, 10,
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {navsatCallback(msg);});

  if (set_origin_) {
    origin_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / origin_pub_rate)),
      [this]() {
        if (origin_set_) {origin_pub_->publish(origin_navsat_);}
      });
  }

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for fix...");
}

void NavsatPreprocessorNode::originCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!origin_set_ && msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    try {
      geographic_msgs::msg::GeoPoint pt;
      pt.latitude = msg->latitude;
      pt.longitude = msg->longitude;
      pt.altitude = msg->altitude;
      origin_utm_ = geodesy::UTMPoint(pt);
      origin_navsat_ = *msg;
      origin_set_ = true;

      RCLCPP_INFO(
        get_logger(), "GPS Origin Received: Lat %.6f, Lon %.6f (UTM Zone %d%c)",
        origin_navsat_.latitude, origin_navsat_.longitude, origin_utm_.zone,
        origin_utm_.band);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Origin set failed: %s", e.what());
    }
  }
}

void NavsatPreprocessorNode::navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received NavSatFix with no fix.");
    return;
  }

  if (!origin_set_) {
    if (!set_origin_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for origin from external source...");
      return;
    }

    try {
      geographic_msgs::msg::GeoPoint pt;
      pt.latitude = msg->latitude;
      pt.longitude = msg->longitude;
      pt.altitude = msg->altitude;
      origin_utm_ = geodesy::UTMPoint(pt);
      origin_navsat_ = *msg;
      origin_set_ = true;

      RCLCPP_INFO(
        get_logger(), "GPS Origin Set: Lat %.6f, Lon %.6f (UTM Zone %d%c)",
        origin_navsat_.latitude, origin_navsat_.longitude, origin_utm_.zone,
        origin_utm_.band);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Origin set failed: %s", e.what());
    }
    return;
  }

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = map_frame_;
  odom_msg.child_frame_id = msg->header.frame_id;

  if (convertToEnu(msg, odom_msg)) {
    odom_pub_->publish(odom_msg);
  }
}

bool NavsatPreprocessorNode::convertToEnu(
  const sensor_msgs::msg::NavSatFix::SharedPtr & msg,
  nav_msgs::msg::Odometry & odom_msg)
{
  try {
    geographic_msgs::msg::GeoPoint pt;
    pt.latitude = msg->latitude;
    pt.longitude = msg->longitude;
    pt.altitude = msg->altitude;
    geodesy::UTMPoint current_utm(pt);

    if (current_utm.zone != origin_utm_.zone || current_utm.band != origin_utm_.band) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "UTM Zone mismatch (%d%c vs %d%c).", current_utm.zone,
        current_utm.band, origin_utm_.zone, origin_utm_.band);
      return false;
    }

    odom_msg.pose.pose.position.x = current_utm.easting - origin_utm_.easting;
    odom_msg.pose.pose.position.y = current_utm.northing - origin_utm_.northing;
    odom_msg.pose.pose.position.z = current_utm.altitude - origin_utm_.altitude;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    if (msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unknown covariance type.");
      return false;
    }

    const auto & cov = msg->position_covariance;
    odom_msg.pose.covariance[0] = cov[0];
    odom_msg.pose.covariance[1] = cov[1];
    odom_msg.pose.covariance[2] = cov[2];
    odom_msg.pose.covariance[6] = cov[3];
    odom_msg.pose.covariance[7] = cov[4];
    odom_msg.pose.covariance[8] = cov[5];
    odom_msg.pose.covariance[12] = cov[6];
    odom_msg.pose.covariance[13] = cov[7];
    odom_msg.pose.covariance[14] = cov[8];

    odom_msg.pose.covariance[21] = 1e9;
    odom_msg.pose.covariance[28] = 1e9;
    odom_msg.pose.covariance[35] = 1e9;

    odom_msg.twist.covariance[0] = -1.0;

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "UTM conversion failed: %s",
      e.what());
    return false;
  }
}

}  // namespace coug_fgo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<coug_fgo::NavsatPreprocessorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
