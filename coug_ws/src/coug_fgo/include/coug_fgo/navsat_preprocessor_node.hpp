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
 * @brief ROS 2 node for preprocessing NavSatFix messages into ENU odometry.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <math.h>
#include <geodesy/utm.h>

#include <string>
#include <vector>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <coug_fgo/navsat_preprocessor_parameters.hpp>

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

  // --- State ---
  bool origin_set_ = false;
  sensor_msgs::msg::NavSatFix origin_navsat_;
  geodesy::UTMPoint origin_utm_;

  bool collecting_samples_ = false;
  double start_collection_time_ = 0.0;
  std::vector<sensor_msgs::msg::NavSatFix> gps_samples_;

  // --- ROS Interfaces ---
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr origin_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
  rclcpp::TimerBase::SharedPtr origin_timer_;

  // --- Parameters ---
  std::shared_ptr<navsat_preprocessor_node::ParamListener> param_listener_;
  navsat_preprocessor_node::Params params_;
};

}  // namespace coug_fgo
