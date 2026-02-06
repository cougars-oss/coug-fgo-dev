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
 * @file test_dvl_factor.cpp
 * @brief Unit tests for dvl_factor.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <boost/bind/bind.hpp>

#include "coug_fgo/factors/dvl_factor.hpp"

/**
 * @brief Test the error evaluation logic of the DvlFactor.
 *
 * Computes the residual error between the DVL measurement (body-frame velocity)
 * and the estimated velocity (converted from world to body frame via pose orientation).
 *
 * Cases tested:
 * 1.  **Identity**: Everything aligned. Zero error.
 * 2.  **Rotation**: Vehicle rotated, sensor aligned.
 * 3.  **Error Check**: Verifies non-zero error magnitude.
 */
TEST(DvlFactorTest, ErrorEvaluation) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::Key velKey = gtsam::symbol_shorthand::V(1);
  gtsam::Vector3 measured_vel(1.0, 0.0, 0.0);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  coug_fgo::factors::DvlFactor factor(poseKey, velKey, measured_vel, model);

  // Case 1: Identity
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor.evaluateError(gtsam::Pose3::Identity(), gtsam::Vector3(1, 0, 0)), 1e-9));

  // Case 2: Rotation
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3(0, 0, 0));
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor.evaluateError(pose, gtsam::Vector3(0, 1, 0)), 1e-9));

  // Case 3: Error Check
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3(1, 0, 0),
      factor.evaluateError(gtsam::Pose3::Identity(), gtsam::Vector3(2, 0, 0)), 1e-9));
}

/**
 * @brief Verify Jacobians of the DvlFactor using numerical differentiation.
 *
 * Validates the analytical Jacobians with respect to:
 * 1.  **Pose**: Orientation affects the world-to-body projection.
 * 2.  **Velocity**: Linear relationship.
 */
TEST(DvlFactorTest, Jacobians) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::Key velKey = gtsam::symbol_shorthand::V(1);
  coug_fgo::factors::DvlFactor factor(poseKey, velKey, gtsam::Vector3(1.0, 0.5, -0.2),
    gtsam::noiseModel::Isotropic::Sigma(3, 0.1));

  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1, 2, 3));
  gtsam::Vector3 vel_world(1.5, -0.5, 0.2);

  gtsam::Matrix expectedH1 = gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3,
      gtsam::Vector3>(
    boost::bind(
      &coug_fgo::factors::DvlFactor::evaluateError, &factor,
      boost::placeholders::_1, boost::placeholders::_2, boost::none, boost::none),
    pose, vel_world, 1e-5);
  gtsam::Matrix expectedH2 = gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3,
      gtsam::Vector3>(
    boost::bind(
      &coug_fgo::factors::DvlFactor::evaluateError, &factor,
      boost::placeholders::_1, boost::placeholders::_2, boost::none, boost::none),
    pose, vel_world, 1e-5);

  gtsam::Matrix actualH1, actualH2;
  factor.evaluateError(pose, vel_world, actualH1, actualH2);
  EXPECT_TRUE(gtsam::assert_equal(expectedH1, actualH1, 1e-5));
  EXPECT_TRUE(gtsam::assert_equal(expectedH2, actualH2, 1e-5));
}
