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
 * @file test_ahrs_factor.cpp
 * @brief Unit tests for ahrs_factor.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <boost/bind/bind.hpp>

#include "coug_fgo/factors/ahrs_factor.hpp"

/**
 * @brief Test the error evaluation logic of the CustomAHRSYawFactorArm.
 *
 * Verifies that the factor correctly accounts for the mounting rotation (offset)
 * between the vehicle's body frame and the AHRS sensor frame.
 *
 * Cases tested:
 * 1.  **Identity**: Everything aligned. Zero error.
 * 2.  **Rotation**: Vehicle rotated, sensor aligned.
 * 3.  **Mounting/Lever Arm**: Sensor offset relative to body.
 * 4.  **Combined**: Vehicle rotated + Sensor offset.
 * 5.  **Error Check**: Verifies non-zero error magnitude.
 */
TEST(CustomAHRSYawFactorArmTest, ErrorEvaluation) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);

  // Case 1: Identity
  coug_fgo::factors::CustomAHRSYawFactorArm factor1(poseKey, gtsam::Rot3::Identity(),
    gtsam::Rot3::Identity(), 0.0, model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector1::Zero(),
      factor1.evaluateError(gtsam::Pose3::Identity()), 1e-9));

  // Case 2: Rotation
  gtsam::Pose3 pose_rot = gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3());
  coug_fgo::factors::CustomAHRSYawFactorArm factor_rot(poseKey, gtsam::Rot3::Yaw(M_PI_2),
    gtsam::Rot3::Identity(), 0.0, model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector1::Zero(),
      factor_rot.evaluateError(pose_rot), 1e-9));

  // Case 3: Mounting/Lever Arm
  coug_fgo::factors::CustomAHRSYawFactorArm factor2(poseKey, gtsam::Rot3::Yaw(M_PI_2),
    gtsam::Rot3::Yaw(M_PI_2), 0.0, model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector1::Zero(),
      factor2.evaluateError(gtsam::Pose3::Identity()), 1e-9));

  // Case 4: Combined
  coug_fgo::factors::CustomAHRSYawFactorArm factor_comb(poseKey, gtsam::Rot3::Yaw(M_PI),
    gtsam::Rot3::Yaw(M_PI_2), 0.0, model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector1::Zero(),
      factor_comb.evaluateError(gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3())), 1e-9));

  // Case 5: Error Check
  double angle = 0.174533;
  gtsam::Vector error =
    factor2.evaluateError(gtsam::Pose3(gtsam::Rot3::Yaw(angle), gtsam::Point3()));
  EXPECT_NEAR(error[0], angle, 1e-5);
}

/**
 * @brief Verify Jacobians of the CustomAHRSYawFactorArm using numerical differentiation.
 *
 * Validates the analytical Jacobians with respect to:
 * 1.  **Pose**: Orientation affects the error (rotational difference).
 */
TEST(CustomAHRSYawFactorArmTest, Jacobians) {
  coug_fgo::factors::CustomAHRSYawFactorArm factor(gtsam::symbol_shorthand::X(1),
    gtsam::Rot3::Ypr(0.5, 0.1, -0.1),
    gtsam::Rot3::Ypr(0.1, 0, 0), 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 0.1));
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::Ypr(0.4, 0.05, -0.05), gtsam::Point3(1, 1, 1));

  gtsam::Matrix expectedH = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    boost::bind(
      &coug_fgo::factors::CustomAHRSYawFactorArm::evaluateError, &factor,
      boost::placeholders::_1, boost::none), pose, 1e-5);

  gtsam::Matrix actualH;
  factor.evaluateError(pose, actualH);
  EXPECT_TRUE(gtsam::assert_equal(expectedH, actualH, 1e-5));
  EXPECT_EQ(actualH.rows(), 1);
  EXPECT_EQ(actualH.cols(), 6);
}
