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
 * @file hsd_commander_node.cpp
 * @brief Implementation of the HsdCommanderNode.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#include "coug_navigation/hsd_commander_node.hpp"

#include <cmath>

namespace coug_navigation
{

HsdCommanderNode::HsdCommanderNode()
: Node("hsd_commander_node")
{
  RCLCPP_INFO(get_logger(), "Starting HSD Commander Node...");

  param_listener_ = std::make_shared<hsd_commander_node::ParamListener>(
    get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  heading_pub_ = create_publisher<std_msgs::msg::Float64>(params_.heading_topic, 10);
  speed_pub_ = create_publisher<std_msgs::msg::Float64>(params_.speed_topic, 10);
  depth_pub_ = create_publisher<std_msgs::msg::Float64>(params_.depth_topic, 10);

  waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    params_.waypoint_topic, 10,
    std::bind(&HsdCommanderNode::waypointCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    params_.odom_topic, 10,
    std::bind(&HsdCommanderNode::odomCallback, this, std::placeholders::_1));

  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&HsdCommanderNode::checkOdomTimeout, this));

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for mission...");
}

void HsdCommanderNode::waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty mission. Stopping mission.");
    stopMission();
    return;
  }

  waypoints_ = msg->poses;
  current_waypoint_index_ = 0;
  state_ = MissionState::ACTIVE;
  previous_distance_ = -1.0;

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

void HsdCommanderNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_time_ = this->get_clock()->now();

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

void HsdCommanderNode::processWaypointLogic(double current_x, double current_y)
{
  const auto & target = waypoints_[current_waypoint_index_];
  double distance = calculateDistance(current_x, current_y, target.position.x, target.position.y);

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
  double heading = std::atan2(dy, dx) * 180.0 / M_PI;

  publishCommands(heading, params_.desired_speed_rpm, target.position.z);
}

double HsdCommanderNode::calculateDistance(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

void HsdCommanderNode::checkOdomTimeout()
{
  if (state_ == MissionState::IDLE || last_odom_time_.nanoseconds() == 0) {
    return;
  }

  auto now = this->get_clock()->now();
  if ((now - last_odom_time_).seconds() > params_.odom_timeout_sec) {
    RCLCPP_ERROR(get_logger(), "Odometry timeout. Stopping mission.");
    stopMission();
  }
}

void HsdCommanderNode::publishCommands(double heading_deg, double speed_rpm, double depth_m)
{
  std_msgs::msg::Float64 msg;

  msg.data = heading_deg;
  heading_pub_->publish(msg);

  msg.data = speed_rpm;
  speed_pub_->publish(msg);

  msg.data = depth_m;
  depth_pub_->publish(msg);
}

void HsdCommanderNode::stopMission()
{
  state_ = MissionState::IDLE;
  waypoints_.clear();
  publishCommands(0.0, 0.0, 0.0);
}

}  // namespace coug_navigation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<coug_navigation::HsdCommanderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
