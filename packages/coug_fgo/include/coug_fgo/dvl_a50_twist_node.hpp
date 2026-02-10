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
 * @file dvl_a50_twist_node.hpp
 * @brief ROS 2 node that converts raw DVL velocity data to a twist message.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <dvl_msgs/msg/dvl.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <coug_fgo/dvl_a50_twist_parameters.hpp>

namespace coug_fgo
{

/**
 * @class DvlA50TwistNode
 * @brief Converts DVL messages to TwistWithCovarianceStamped messages.
 *
 * This node subscribes to DVL messages from a Waterlinked A50 DVL and
 * republishes the velocity data as a standard ROS 2 twist message.
 */
class DvlA50TwistNode : public rclcpp::Node
{
public:
  /**
   * @brief DvlA50TwistNode constructor.
   * @param options The node options.
   */
  explicit DvlA50TwistNode(const rclcpp::NodeOptions & options);

protected:
  /**
   * @brief Callback for receiving new DVL data.
   * @param msg The incoming DVL message.
   */
  void dvlCallback(const dvl_msgs::msg::DVL::SharedPtr msg);

  /**
   * @brief Converts a DVL message to a TwistWithCovarianceStamped message.
   * @param msg The incoming DVL message.
   * @return The converted TwistWithCovarianceStamped message.
   */
  geometry_msgs::msg::TwistWithCovarianceStamped convertToTwist(
    const dvl_msgs::msg::DVL::SharedPtr msg);

  /**
   * @brief Diagnostic task to report the status of the DVL data.
   * @param stat The diagnostic status wrapper.
   */
  void checkDvlStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // --- ROS Interfaces ---
  rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<dvl_a50_twist_node::ParamListener> param_listener_;
  dvl_a50_twist_node::Params params_;

  // --- State ---
  std::atomic<double> last_dvl_time_{0.0};
  std::atomic<bool> last_velocity_valid_{false};
};

}  // namespace coug_fgo
