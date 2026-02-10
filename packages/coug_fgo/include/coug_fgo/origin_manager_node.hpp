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
 * @file origin_manager_node.hpp
 * @brief ROS 2 node for AUV fleet origin management.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <math.h>
#include <geodesy/utm.h>

#include <atomic>
#include <string>
#include <vector>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <coug_fgo/origin_manager_parameters.hpp>

namespace coug_fgo
{

/**
 * @class OriginManagerNode
 * @brief Manages the geographic origin of the AUV fleet.
 *
 * This node converts global geographic coordinates (latitude, longitude, altitude)
 * into a local East-North-Up (ENU) frame relative to a set origin. It also handles
 * origin handshakes in multi-agent fleet scenarios.
 */
class OriginManagerNode : public rclcpp::Node
{
public:
  /**
   * @brief OriginManagerNode constructor.
   * @param options The node options.
   */
  explicit OriginManagerNode(const rclcpp::NodeOptions & options);

protected:
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

  // --- Diagnostics ---
  /**
   * @brief Diagnostic task to report the status of the GPS origin.
   * @param stat The diagnostic status wrapper.
   */
  void checkOriginStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Diagnostic task to report the status of the GPS fix.
   * @param stat The diagnostic status wrapper.
   */
  void checkNavSatFix(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // --- State ---
  std::atomic<bool> origin_set_{false};
  sensor_msgs::msg::NavSatFix origin_navsat_;
  geodesy::UTMPoint origin_utm_;

  std::atomic<double> last_navsat_time_{0.0};
  std::atomic<int> last_fix_status_{-1};

  bool collecting_samples_ = false;
  double start_collection_time_ = 0.0;
  std::vector<sensor_msgs::msg::NavSatFix> gps_samples_;

  // --- ROS Interfaces ---
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr origin_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
  rclcpp::TimerBase::SharedPtr origin_timer_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<origin_manager_node::ParamListener> param_listener_;
  origin_manager_node::Params params_;
};

}  // namespace coug_fgo
