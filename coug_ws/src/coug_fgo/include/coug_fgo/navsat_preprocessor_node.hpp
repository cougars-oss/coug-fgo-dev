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
 * @file navsat_preprocessor_node.hpp
 * @brief ROS2 node for preprocessing NavSatFix messages into ENU odometry.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <math.h>
#include <geodesy/utm.h>

#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace coug_fgo
{

/**
 * @class NavsatPreprocessorNode
 * @brief Preprocesses NavSatFix measurements into local ENU coordinates.
 *
 * This node converts global geographic coordinates (latitude, longitude, altitude)
 * into a local East-North-Up (ENU) frame relative to a set origin. It also handles
 * the publication and acquisition of the geographic origin.
 */
class NavsatPreprocessorNode : public rclcpp::Node
{
public:
  /**
   * @brief NavsatPreprocessorNode constructor.
   */
  NavsatPreprocessorNode();

private:
  // --- Logic ---
  /**
   * @brief Callback for incoming NavSatFix messages.
   * @param msg The incoming NavSatFix message.
   */
  void navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  /**
   * @brief Callback for the external geographic origin.
   * @param msg The incoming origin NavSatFix message.
   */
  void originCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  /**
   * @brief Converts a NavSatFix message to ENU odometry.
   * @param msg The incoming NavSatFix message.
   * @param odom_msg The output Odometry message.
   * @return True if conversion was successful, false otherwise.
   */
  bool convertToEnu(
    const sensor_msgs::msg::NavSatFix::SharedPtr & msg,
    nav_msgs::msg::Odometry & odom_msg);

  // --- ROS Interfaces ---
  /// ENU odometry publisher.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  /// Origin publisher.
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr origin_pub_;
  /// GPS fix subscriber.
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
  /// External origin subscriber.
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
  /// Timer for periodic origin publication.
  rclcpp::TimerBase::SharedPtr origin_timer_;

  // --- State ---
  /// Flag indicating if the origin has been established.
  bool origin_set_ = false;
  /// The geographic coordinates of the origin.
  sensor_msgs::msg::NavSatFix origin_navsat_;
  /// The UTM coordinates of the origin.
  geodesy::UTMPoint origin_utm_;

  /// Flag indicating if we are currently collecting samples for averaging.
  bool collecting_samples_ = false;
  /// Start time of the collection period.
  double start_collection_time_ = 0.0;
  /// Buffer for GPS samples.
  std::vector<sensor_msgs::msg::NavSatFix> gps_samples_;

  // --- Parameters ---
  /// If true, this node establishes and publishes the origin.
  /// If false, it subscribes to an external origin topic.
  bool set_origin_;
  /// The local ENU map frame ID.
  std::string map_frame_;
  /// Duration to average GPS measurements before setting origin.
  double initialization_duration_;
};

}  // namespace coug_fgo
