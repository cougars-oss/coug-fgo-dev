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
 * @file test_depth_factor.cpp
 * @brief Unit tests for depth_factor.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <boost/bind/bind.hpp>

#include "coug_fgo/factors/depth_factor.hpp"

/**
 * @brief Test the error evaluation logic of the CustomDepthFactorArm.
 *
 * Verifies the factor's residual calculation: `error = measured_depth - (pose.z + lever_arm_z)`.
 *
 * Cases tested:
 * 1.  **Identity**: Everything aligned. Zero error.
 * 2.  **Rotation**: Vehicle rotated, sensor aligned.
 * 3.  **Mounting/Lever Arm**: Sensor offset relative to body.
 * 4.  **Combined**: Vehicle rotated + Sensor offset.
 * 5.  **Error Check**: Verifies non-zero error magnitude.
 */
TEST(CustomDepthFactorArmTest, ErrorEvaluation) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);

  // Case 1: Identity
  coug_fgo::factors::CustomDepthFactorArm factor1(poseKey, 5.0, gtsam::Pose3::Identity(), model);
  EXPECT_NEAR(
    factor1.evaluateError(
      gtsam::Pose3(
        gtsam::Rot3(), gtsam::Point3(
          0, 0,
          5)))[0], 0.0, 1e-9);

  // Case 2: Rotation
  EXPECT_NEAR(
    factor1.evaluateError(
      gtsam::Pose3(
        gtsam::Rot3::Rx(M_PI), gtsam::Point3(
          0, 0,
          5)))[0], 0.0, 1e-9);

  // Case 3: Mounting/Lever Arm
  coug_fgo::factors::CustomDepthFactorArm factor2(poseKey, 5.0,
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1)), model);
  EXPECT_NEAR(
    factor2.evaluateError(
      gtsam::Pose3(
        gtsam::Rot3(), gtsam::Point3(
          0, 0,
          4)))[0], 0.0, 1e-9);

  // Case 4: Combined
  EXPECT_NEAR(
    factor2.evaluateError(
      gtsam::Pose3(
        gtsam::Rot3::Rx(M_PI), gtsam::Point3(
          0, 0,
          6)))[0], 0.0,
    1e-9);

  // Case 5: Error Check
  EXPECT_NEAR(
    factor1.evaluateError(
      gtsam::Pose3(
        gtsam::Rot3(), gtsam::Point3(
          0, 0,
          6)))[0], 1.0, 1e-9);
}

/**
 * @brief Verify Jacobians of the CustomDepthFactorArm using numerical differentiation.
 *
 * Validates the analytical Jacobians with respect to:
 * 1.  **Pose**: Z-component relates to depth measurement.
 */
TEST(CustomDepthFactorArmTest, Jacobians) {
  coug_fgo::factors::CustomDepthFactorArm factor(gtsam::symbol_shorthand::X(1), 5.0,
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.5, 0.5, 0.5)),
    gtsam::noiseModel::Isotropic::Sigma(1, 0.1));
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1, 2, 4));

  gtsam::Matrix expectedH = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    boost::bind(
      &coug_fgo::factors::CustomDepthFactorArm::evaluateError, &factor,
      boost::placeholders::_1, boost::none), pose, 1e-5);

  gtsam::Matrix actualH;
  factor.evaluateError(pose, actualH);
  EXPECT_TRUE(gtsam::assert_equal(expectedH, actualH, 1e-5));
}
