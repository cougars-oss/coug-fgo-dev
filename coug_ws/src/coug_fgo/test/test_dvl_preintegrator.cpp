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
 * @file test_dvl_preintegrator.cpp
 * @brief Unit tests for dvl_preintegrator.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>

#include "coug_fgo/utils/dvl_preintegrator.hpp"

/**
 * @brief Test fixture for DVLPreintegrator tests.
 */
class DVLPreintegratorTest : public ::testing::Test
{
protected:
  coug_fgo::utils::DVLPreintegrator integrator;
  gtsam::Matrix3 measured_cov = gtsam::Matrix3::Identity() * 0.01;
};

/**
 * @brief Verify initialization state.
 */
TEST_F(DVLPreintegratorTest, Initialization) {
  EXPECT_TRUE(integrator.delta().isZero());
  EXPECT_TRUE(integrator.covariance().isZero());
}

/**
 * @brief Verify integration of stationary measurements.
 */
TEST_F(DVLPreintegratorTest, StationaryIntegration) {
  gtsam::Vector3 vel(0, 0, 0);
  gtsam::Rot3 orient = gtsam::Rot3::Identity();
  for (int i = 0; i < 10; ++i) {
    integrator.integrateMeasurement(vel, orient, 0.1, measured_cov);
  }
  EXPECT_TRUE(integrator.delta().isZero());
}

/**
 * @brief Verify constant velocity integration.
 */
TEST_F(DVLPreintegratorTest, ConstantVelocityX) {
  gtsam::Vector3 vel(1.0, 0, 0);
  gtsam::Rot3 orient = gtsam::Rot3::Identity();
  for (int i = 0; i < 10; ++i) {
    integrator.integrateMeasurement(vel, orient, 0.1, measured_cov);
  }

  gtsam::Vector3 delta = integrator.delta();
  EXPECT_NEAR(delta.x(), 1.0, 1e-6);
  EXPECT_NEAR(delta.y(), 0.0, 1e-6);
}

/**
 * @brief Verify integration with orientation changes.
 */
TEST_F(DVLPreintegratorTest, RotatedIntegration) {
  gtsam::Vector3 vel(1.0, 0, 0);
  gtsam::Rot3 orient = gtsam::Rot3::Yaw(M_PI_2);

  integrator.integrateMeasurement(vel, orient, 1.0, measured_cov);

  gtsam::Vector3 delta = integrator.delta();
  EXPECT_NEAR(delta.x(), 0.0, 1e-6);
  EXPECT_NEAR(delta.y(), 1.0, 1e-6);
}

/**
 * @brief Verify reset functionality.
 */
TEST_F(DVLPreintegratorTest, Reset) {
  integrator.integrateMeasurement(gtsam::Vector3(1, 0, 0), gtsam::Rot3(), 1.0, measured_cov);
  EXPECT_FALSE(integrator.delta().isZero());

  integrator.reset(gtsam::Rot3::Identity());
  EXPECT_TRUE(integrator.delta().isZero());
  EXPECT_TRUE(integrator.covariance().isZero());
}
