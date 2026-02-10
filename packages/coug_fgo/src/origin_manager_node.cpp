// Copyright (c) 2026 BYU FROST Lab
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
 * @file origin_manager_node.cpp
 * @brief Implementation of the OriginManagerNode.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#include "coug_fgo/origin_manager_node.hpp"

#include <geodesy/wgs84.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geographic_msgs/msg/geo_point.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace coug_fgo
{

OriginManagerNode::OriginManagerNode(const rclcpp::NodeOptions & options)
: Node("origin_manager_node", options),
  diagnostic_updater_(this)
{
  RCLCPP_INFO(get_logger(), "Starting Origin Manager Node...");

  param_listener_ = std::make_shared<origin_manager_node::ParamListener>(
    get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(params_.odom_output_topic, 10);

  if (params_.set_origin) {
    origin_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(params_.origin_topic, 10);
  } else {
    origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      params_.origin_topic, 10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {originCallback(msg);});
  }

  if (params_.use_parameter_origin && params_.set_origin) {
    try {
      geographic_msgs::msg::GeoPoint pt;
      pt.latitude = params_.parameter_origin.latitude;
      pt.longitude = params_.parameter_origin.longitude;
      pt.altitude = params_.parameter_origin.altitude;
      origin_utm_ = geodesy::UTMPoint(pt);

      origin_navsat_.header.frame_id = params_.map_frame;
      origin_navsat_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      origin_navsat_.latitude = params_.parameter_origin.latitude;
      origin_navsat_.longitude = params_.parameter_origin.longitude;
      origin_navsat_.altitude = params_.parameter_origin.altitude;

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
    params_.input_topic, 10,
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {navsatCallback(msg);});

  if (params_.set_origin) {
    origin_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / params_.origin_pub_rate)),
      [this]() {
        if (origin_set_) {origin_pub_->publish(origin_navsat_);}
      });
  }

  // --- ROS Diagnostics ---
  std::string ns = this->get_namespace();
  std::string clean_ns = (ns == "/") ? "" : ns;
  diagnostic_updater_.setHardwareID(clean_ns + "/origin_manager_node");

  std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";

  std::string origin_task = prefix + "GPS Origin";
  diagnostic_updater_.add(origin_task, this, &OriginManagerNode::checkOriginStatus);

  std::string fix_task = prefix + "GPS Fix";
  diagnostic_updater_.add(fix_task, this, &OriginManagerNode::checkNavSatFix);

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for fix...");
}

void OriginManagerNode::originCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!origin_set_ && msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
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

void OriginManagerNode::navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  last_navsat_time_ = this->get_clock()->now().seconds();
  last_fix_status_ = msg->status.status;

  if (params_.simulate_dropout && params_.dropout_frequency > 0) {
    double current_time = this->get_clock()->now().seconds();
    double cycle_period = 1.0 / params_.dropout_frequency;
    if (fmod(current_time, cycle_period) < params_.dropout_duration) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), (int)(cycle_period * 1000),
        "Simulating GPS dropout...");
      return;
    }
  }

  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received NavSatFix with no fix.");
    return;
  }

  if (!origin_set_) {
    if (!params_.set_origin) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for origin from external source...");
      return;
    }

    if (!collecting_samples_) {
      collecting_samples_ = true;
      start_collection_time_ = this->get_clock()->now().seconds();
      gps_samples_.clear();
      RCLCPP_INFO(
        get_logger(), "Starting GPS origin averaging (%.1fs)...",
        params_.initialization_duration);
    }

    gps_samples_.push_back(*msg);

    double elapsed = this->get_clock()->now().seconds() - start_collection_time_;
    if (elapsed < params_.initialization_duration) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Averaging GPS data (%.2fs / %.2fs)...", elapsed,
        params_.initialization_duration);
      return;
    }

    // Average GPS
    double lat_sum = 0.0;
    double lon_sum = 0.0;
    double alt_sum = 0.0;
    for (const auto & sample : gps_samples_) {
      lat_sum += sample.latitude;
      lon_sum += sample.longitude;
      alt_sum += sample.altitude;
    }
    double n = static_cast<double>(gps_samples_.size());

    try {
      geographic_msgs::msg::GeoPoint pt;
      pt.latitude = lat_sum / n;
      pt.longitude = lon_sum / n;
      pt.altitude = alt_sum / n;
      origin_utm_ = geodesy::UTMPoint(pt);

      origin_navsat_ = gps_samples_.back();
      origin_navsat_.latitude = pt.latitude;
      origin_navsat_.longitude = pt.longitude;
      origin_navsat_.altitude = pt.altitude;
      origin_set_ = true;

      RCLCPP_INFO(
        get_logger(),
        "GPS Origin Set (Averaged %d samples): Lat %.6f, Lon %.6f (UTM Zone %d%c)",
        (int)n, origin_navsat_.latitude, origin_navsat_.longitude, origin_utm_.zone,
        origin_utm_.band);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Origin set failed: %s", e.what());
      collecting_samples_ = false;
    }
    return;
  }

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = params_.map_frame;

  if (params_.use_parameter_child_frame) {
    odom_msg.child_frame_id = params_.parameter_child_frame;
  } else {
    odom_msg.child_frame_id = msg->header.frame_id;
  }

  if (convertToEnu(msg, odom_msg)) {
    odom_pub_->publish(odom_msg);
  }
}

bool OriginManagerNode::convertToEnu(
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

void OriginManagerNode::checkOriginStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (origin_set_) {
    stat.add("Origin Zone", std::to_string(origin_utm_.zone) + origin_utm_.band);
    stat.add("Origin Latitude", origin_navsat_.latitude);
    stat.add("Origin Longitude", origin_navsat_.longitude);
    stat.add("Origin Altitude", origin_navsat_.altitude);

    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Origin successfully set.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for origin.");
  }
}

void OriginManagerNode::checkNavSatFix(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "GPS fix acquired.");

  stat.add("Fix Status", last_fix_status_);

  double time_since =
    (last_navsat_time_ > 0.0) ? (this->get_clock()->now().seconds() - last_navsat_time_) : -1.0;
  stat.add("Time Since Last (s)", time_since);

  if (time_since > params_.timeout_threshold || last_navsat_time_ == 0.0) {
    stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "GPS is offline.");
  }
  if (last_fix_status_ == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No GPS fix acquired.");
  }
}

}  // namespace coug_fgo

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fgo::OriginManagerNode)
