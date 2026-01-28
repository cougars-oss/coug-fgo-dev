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
 * @file hsd_commander_node.hpp
 * @brief ROS 2 node for HSD (Heading, Speed, Depth) command generation and waypoint navigation.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <string>
#include <vector>
#include <memory>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <coug_navigation/hsd_commander_parameters.hpp>

namespace coug_navigation
{

/**
 * @class HsdCommanderNode
 * @brief High-level mission commander for AUV waypoint navigation.
 *
 * This node manages AUV mission execution by following a list of waypoints.
 * It uses a capture radius to detect waypoint arrival and a slip radius to handle overshoots.
 * It publishes Heading, Speed, and Depth (HSD) commands to the control system.
 */
class HsdCommanderNode : public rclcpp::Node
{
public:
  /**
   * @brief HsdCommanderNode constructor.
   */
  HsdCommanderNode();

private:
  // --- Logic ---
  /**
   * @brief Callback for receiving a new list of waypoints.
   * @param msg The incoming PoseArray message containing waypoints.
   */
  void waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  /**
   * @brief Callback for odometry updates. Main control loop.
   * @param msg The incoming Odometry message.
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Checks if odometry data has stopped arriving.
   */
  void checkOdomTimeout();

  /**
   * @brief Publishes HSD commands to the vehicle.
   * @param heading_deg Desired heading in degrees.
   * @param speed_rpm Desired speed in RPM.
   * @param depth_m Desired depth in meters.
   */
  void publishCommands(double heading_deg, double speed_rpm, double depth_m);

  /**
   * @brief Stops the current mission and halts the vehicle.
   */
  void stopMission();

  /**
   * @brief Calculates Euclidean distance between two 2D points.
   * @param x1 First point X.
   * @param y1 First point Y.
   * @param x2 Second point X.
   * @param y2 Second point Y.
   * @return Distance in meters.
   */
  double calculateDistance(double x1, double y1, double x2, double y2);

  /**
   * @brief Logic to check waypoint arrival and update mission state.
   * @param current_x Current X position.
   * @param current_y Current Y position.
   */
  void processWaypointLogic(double current_x, double current_y);

  // --- State ---
  enum class MissionState
  {
    IDLE,
    ACTIVE
  };

  MissionState state_ = MissionState::IDLE;

  std::vector<geometry_msgs::msg::Pose> waypoints_;
  size_t current_waypoint_index_ = 0;
  rclcpp::Time last_odom_time_;
  double previous_distance_ = -1.0;

  // --- ROS Interfaces ---
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // --- Parameters ---
  std::shared_ptr<hsd_commander_node::ParamListener> param_listener_;
  hsd_commander_node::Params params_;
};

}  // namespace coug_navigation
