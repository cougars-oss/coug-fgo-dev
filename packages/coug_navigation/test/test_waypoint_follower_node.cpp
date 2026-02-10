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
 * @file test_waypoint_follower_node.cpp
 * @brief Unit tests for waypoint_follower_node.hpp.
 * @author Nelson Durrant
 * @date Feb 2026
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "coug_navigation/waypoint_follower_node.hpp"

using coug_navigation::WaypointFollowerNode;

/**
 * @class TestWaypointFollowerNode
 * @brief Harness to expose protected WaypointFollowerNode members for testing.
 */
class TestWaypointFollowerNode : public WaypointFollowerNode
{
public:
  using WaypointFollowerNode::WaypointFollowerNode;

  // Expose protected methods
  using WaypointFollowerNode::calculateDistance;
  using WaypointFollowerNode::processWaypointLogic;
  using WaypointFollowerNode::stopMission;

  // Expose protected state
  using WaypointFollowerNode::state_;
  using WaypointFollowerNode::waypoints_;
  using WaypointFollowerNode::current_waypoint_index_;
  using WaypointFollowerNode::params_;
};

/**
 * @class WaypointFollowerNodeTest
 * @brief Test fixture for WaypointFollowerNode tests.
 */
class WaypointFollowerNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    node = std::make_shared<TestWaypointFollowerNode>(options);
  }

  std::shared_ptr<TestWaypointFollowerNode> node;
};

/**
 * @brief Verify distance calculation.
 */
TEST_F(WaypointFollowerNodeTest, CalculateDistance) {
  EXPECT_NEAR(node->calculateDistance(0.0, 0.0, 3.0, 4.0), 5.0, 1e-6);
  EXPECT_NEAR(node->calculateDistance(-1.0, -1.0, 2.0, 3.0), 5.0, 1e-6);
}

/**
 * @brief Verify waypoint logic and arrival.
 */
TEST_F(WaypointFollowerNodeTest, ProcessWaypointLogic) {
  geometry_msgs::msg::Pose p1;
  p1.position.x = 10.0;
  p1.position.y = 10.0;

  node->params_.capture_radius = 1.5;
  node->params_.slip_radius = 5.0;
  node->waypoints_.push_back(p1);
  node->current_waypoint_index_ = 0;

  node->processWaypointLogic(0.0, 0.0);
  EXPECT_EQ(node->current_waypoint_index_, 0u);

  node->processWaypointLogic(9.0, 9.0);
  EXPECT_EQ(node->current_waypoint_index_, 1u);
}

/**
 * @brief Verify mission stop logic.
 */
TEST_F(WaypointFollowerNodeTest, StopMission) {
  node->state_ = WaypointFollowerNode::MissionState::ACTIVE;
  node->waypoints_.push_back(geometry_msgs::msg::Pose());

  node->stopMission();

  EXPECT_EQ(node->state_, WaypointFollowerNode::MissionState::IDLE);
  EXPECT_TRUE(node->waypoints_.empty());
}
