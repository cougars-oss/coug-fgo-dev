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
 * @file waypoint_follower_node.cpp
 * @brief Implementation of the WaypointFollowerNode.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#include "coug_navigation/waypoint_follower_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace coug_navigation
{

WaypointFollowerNode::WaypointFollowerNode(const rclcpp::NodeOptions & options)
: Node("waypoint_follower_node", options),
  diagnostic_updater_(this)
{
  RCLCPP_INFO(get_logger(), "Starting Waypoint Follower Node...");

  param_listener_ = std::make_shared<waypoint_follower_node::ParamListener>(
    get_node_parameters_interface());
  params_ = param_listener_->get_params();

  std::random_device rd;
  rng_ = std::mt19937(rd());
  speed_dist_ = std::uniform_real_distribution<double>(
    -params_.rw_step_size,
    params_.rw_step_size);

  // --- ROS Interfaces ---
  heading_pub_ = create_publisher<std_msgs::msg::Float64>(params_.heading_topic, 10);
  speed_pub_ = create_publisher<std_msgs::msg::Float64>(params_.speed_topic, 10);
  depth_pub_ = create_publisher<std_msgs::msg::Float64>(params_.depth_topic, 10);

  waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    params_.waypoint_topic, 10,
    std::bind(&WaypointFollowerNode::waypointCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    params_.odom_topic, 10,
    std::bind(&WaypointFollowerNode::odomCallback, this, std::placeholders::_1));

  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&WaypointFollowerNode::checkOdomTimeout, this));

  // --- ROS Diagnostics ---
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/waypoint_follower_node");

    std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";

    std::string mission_task = prefix + "Mission Status";
    diagnostic_updater_.add(mission_task, this, &WaypointFollowerNode::checkMissionStatus);

    std::string odom_task = prefix + "Odometry Link";
    diagnostic_updater_.add(odom_task, this, &WaypointFollowerNode::checkOdometryStatus);
  }

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for mission...");
}


void WaypointFollowerNode::waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty mission. Stopping mission.");
    stopMission();
    return;
  }

  waypoints_ = msg->poses;
  total_waypoints_ = waypoints_.size();
  current_waypoint_index_ = 0;
  state_ = MissionState::ACTIVE;
  previous_distance_ = -1.0;
  current_dist_to_target_ = -1.0;
  current_speed_rpm_ = params_.rw_start_value;

  std::stringstream ss;
  ss << "Started mission with " << msg->poses.size() << " waypoints: ";
  for (size_t i = 0; i < msg->poses.size(); ++i) {
    const auto & p = msg->poses[i].position;
    ss << "[" << i << "]: (" << p.x << ", " << p.y << ", " << p.z << ")";
    if (i < msg->poses.size() - 1) {
      ss << " | ";
    }
  }
  RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
}

void WaypointFollowerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_time_seconds_ = this->get_clock()->now().seconds();

  if (state_ == MissionState::IDLE) {
    return;
  }

  if (current_waypoint_index_ >= waypoints_.size()) {
    RCLCPP_INFO(get_logger(), "Mission complete.");
    stopMission();
    return;
  }

  processWaypointLogic(msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void WaypointFollowerNode::processWaypointLogic(double current_x, double current_y)
{
  const auto & target = waypoints_[current_waypoint_index_];
  double distance = calculateDistance(current_x, current_y, target.position.x, target.position.y);
  current_dist_to_target_ = distance;

  if (distance < params_.capture_radius) {
    current_waypoint_index_++;
    previous_distance_ = -1.0;
    return;
  }

  if (previous_distance_ != -1.0 && distance > previous_distance_ &&
    distance < params_.slip_radius)
  {
    current_waypoint_index_++;
    previous_distance_ = -1.0;
    return;
  }

  previous_distance_ = distance;

  double dx = target.position.x - current_x;
  double dy = target.position.y - current_y;
  double heading = std::atan2(dy, dx) * 180.0 / std::acos(-1.0);

  double speed_cmd = params_.desired_speed_rpm;
  if (params_.enable_rw_speed) {
    current_speed_rpm_ += speed_dist_(rng_);
    current_speed_rpm_ = std::clamp(current_speed_rpm_, 800.0, 1500.0);
    speed_cmd = current_speed_rpm_;
  }

  double depth_cmd = params_.invert_z ? -target.position.z : target.position.z;
  publishCommands(heading, speed_cmd, depth_cmd);
}

double WaypointFollowerNode::calculateDistance(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

void WaypointFollowerNode::checkOdomTimeout()
{
  if (state_ == MissionState::IDLE || last_odom_time_seconds_ == 0.0) {
    return;
  }

  double now = this->get_clock()->now().seconds();
  if ((now - last_odom_time_seconds_) > params_.odom_timeout_sec) {
    RCLCPP_ERROR(get_logger(), "Odometry timeout. Stopping mission.");
    stopMission();
  }
}

void WaypointFollowerNode::publishCommands(double heading_deg, double speed_rpm, double depth_m)
{
  std_msgs::msg::Float64 msg;

  msg.data = heading_deg;
  heading_pub_->publish(msg);

  msg.data = speed_rpm;
  speed_pub_->publish(msg);

  msg.data = depth_m;
  depth_pub_->publish(msg);
}

void WaypointFollowerNode::stopMission()
{
  state_ = MissionState::IDLE;
  waypoints_.clear();
  total_waypoints_ = 0;
  current_dist_to_target_ = 0.0;
  publishCommands(0.0, 0.0, 0.0);
}

void WaypointFollowerNode::checkMissionStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (state_ == MissionState::IDLE) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No mission received.");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Mission received. Navigating to waypoint " + std::to_string(current_waypoint_index_ + 1) +
      "/" + std::to_string(total_waypoints_.load()));

    stat.add("Distance to Target (m)", current_dist_to_target_.load());
  }
}

void WaypointFollowerNode::checkOdometryStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (last_odom_time_seconds_ == 0.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No Odometry received.");
    return;
  }

  double time_since = this->get_clock()->now().seconds() - last_odom_time_seconds_;
  stat.add("Time Since Last (s)", time_since);

  if (time_since > params_.odom_timeout_sec) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Odometry link lost. Mission stopped.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Odometry link online.");
  }
}

}  // namespace coug_navigation

RCLCPP_COMPONENTS_REGISTER_NODE(coug_navigation::WaypointFollowerNode)
