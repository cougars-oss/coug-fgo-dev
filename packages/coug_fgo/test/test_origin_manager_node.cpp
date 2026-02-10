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
 * @file test_origin_manager_node.cpp
 * @brief Unit tests for origin_manager_node.hpp
 * @author Nelson Durrant
 * @date Feb 2026
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "coug_fgo/origin_manager_node.hpp"

using coug_fgo::OriginManagerNode;

/**
 * @class TestOriginManagerNode
 * @brief Harness to expose protected OriginManagerNode members for testing.
 */
class TestOriginManagerNode : public OriginManagerNode
{
public:
  using OriginManagerNode::OriginManagerNode;

  // Expose protected methods
  using OriginManagerNode::navsatCallback;
  using OriginManagerNode::originCallback;
  using OriginManagerNode::convertToEnu;

  // Expose protected state
  using OriginManagerNode::origin_set_;
  using OriginManagerNode::origin_navsat_;
  using OriginManagerNode::origin_utm_;
  using OriginManagerNode::params_;
  using OriginManagerNode::gps_samples_;
};

/**
 * @class OriginManagerNodeTest
 * @brief Test fixture for OriginManagerNode tests.
 */
class OriginManagerNodeTest : public ::testing::Test
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
    node = std::make_shared<TestOriginManagerNode>(options);
  }

  std::shared_ptr<TestOriginManagerNode> node;

  sensor_msgs::msg::NavSatFix::SharedPtr createNavSatMsg(double lat, double lon, double alt)
  {
    auto msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
    msg->header.stamp = rclcpp::Clock().now();
    msg->header.frame_id = "gps_link";
    msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg->latitude = lat;
    msg->longitude = lon;
    msg->altitude = alt;
    msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    msg->position_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    return msg;
  }
};

/**
 * @brief Verify origin setting from external messages.
 */
TEST_F(OriginManagerNodeTest, OriginCallback) {
  auto msg = createNavSatMsg(40.2444, -111.6608, 1400.0);
  node->originCallback(msg);

  EXPECT_TRUE(node->origin_set_);
  EXPECT_NEAR(node->origin_navsat_.latitude, 40.2444, 1e-6);

  auto msg2 = createNavSatMsg(45.0, -120.0, 1000.0);
  node->originCallback(msg2);
  EXPECT_NEAR(node->origin_navsat_.latitude, 40.2444, 1e-6);
}

/**
 * @brief Verify GPS sample averaging and initialization logic.
 */
TEST_F(OriginManagerNodeTest, NavsatCallback) {
  node->params_.set_origin = true;
  node->params_.initialization_duration = 0.5;
  node->origin_set_ = false;

  node->navsatCallback(createNavSatMsg(40.0, -111.0, 1000.0));
  node->navsatCallback(createNavSatMsg(40.2, -111.2, 1010.0));

  rclcpp::sleep_for(std::chrono::milliseconds(600));

  node->navsatCallback(createNavSatMsg(40.1, -111.1, 1005.0));

  EXPECT_TRUE(node->origin_set_);
  EXPECT_NEAR(node->origin_navsat_.latitude, 40.1, 1e-6);
}

/**
 * @brief Verify ENU conversion math and UTM zone safety.
 */
TEST_F(OriginManagerNodeTest, ConvertToEnu) {
  node->originCallback(createNavSatMsg(40.2444, -111.6608, 1400.0));
  auto current_msg = createNavSatMsg(40.2454, -111.6598, 1410.0);
  nav_msgs::msg::Odometry odom;

  EXPECT_TRUE(node->convertToEnu(current_msg, odom));
  EXPECT_GT(odom.pose.pose.position.x, 0.0);
  EXPECT_GT(odom.pose.pose.position.y, 0.0);
  EXPECT_NEAR(odom.pose.pose.position.z, 10.0, 1e-3);

  auto far_msg = createNavSatMsg(21.3069, -157.8583, 0.0);
  EXPECT_FALSE(node->convertToEnu(far_msg, odom));
}
