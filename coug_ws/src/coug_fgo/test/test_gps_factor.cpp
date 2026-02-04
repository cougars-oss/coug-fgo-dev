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
 * @file test_gps_factor.cpp
 * @brief Unit tests for gps_factor.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <boost/bind/bind.hpp>

#include "coug_fgo/factors/gps_factor.hpp"

/**
 * @brief Test the error evaluation logic of the CustomGPS2DFactorArm.
 *
 * Verifies that the factor correctly accounts for the lever arm offset between
 * the vehicle's body origin (Pose) and the GPS antenna location.
 *
 * Cases tested:
 * 1.  **Identity**: Everything aligned. Zero error.
 * 2.  **Rotation**: Vehicle rotated, sensor aligned.
 * 3.  **Mounting/Lever Arm**: Sensor offset relative to body.
 * 4.  **Combined**: Vehicle rotated + Sensor offset.
 * 5.  **Error Check**: Verifies non-zero error magnitude.
 */
TEST(CustomGPS2DFactorArmTest, ErrorEvaluation) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);

  // Case 1: Identity
  coug_fgo::factors::CustomGPS2DFactorArm factor1(poseKey, gtsam::Point3(1, 2, 3),
    gtsam::Pose3::Identity(), model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector2::Zero(),
      factor1.evaluateError(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 2, 3))), 1e-9));

  // Case 2: Rotation
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector2::Zero(),
      factor1.evaluateError(gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3(1, 2, 3))), 1e-9));

  // Case 3: Mounting/Lever Arm
  coug_fgo::factors::CustomGPS2DFactorArm factor2(poseKey, gtsam::Point3(1, 2, 3),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)), model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector2::Zero(),
      factor2.evaluateError(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 2, 3))), 1e-9));

  // Case 4: Combined
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector2::Zero(),
      factor2.evaluateError(gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3(1, 1, 3))), 1e-9));

  // Case 5: Error Check
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector2(1, 0),
      factor1.evaluateError(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 4))), 1e-9));
}

/**
 * @brief Verify Jacobians of the CustomGPS2DFactorArm using numerical differentiation.
 *
 * Validates the analytical Jacobians with respect to:
 * 1.  **Pose**: Position and orientation (due to lever arm).
 */
TEST(CustomGPS2DFactorArmTest, Jacobians) {
  coug_fgo::factors::CustomGPS2DFactorArm factor(gtsam::symbol_shorthand::X(1),
    gtsam::Point3(5, 5, 5),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.2, -0.1, 0.5)),
    gtsam::noiseModel::Isotropic::Sigma(2, 0.1));
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::Ypr(0.5, -0.2, 0.1), gtsam::Point3(4, 5, 6));

  gtsam::Matrix expectedH = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    boost::bind(
      &coug_fgo::factors::CustomGPS2DFactorArm::evaluateError, &factor,
      boost::placeholders::_1, boost::none), pose, 1e-5);

  gtsam::Matrix actualH;
  factor.evaluateError(pose, actualH);
  EXPECT_TRUE(gtsam::assert_equal(expectedH, actualH, 1e-5));
  EXPECT_EQ(actualH.rows(), 2);
  EXPECT_EQ(actualH.cols(), 6);
}
