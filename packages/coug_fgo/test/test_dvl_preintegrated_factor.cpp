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
 * @file test_dvl_preintegrated_factor.cpp
 * @brief Unit tests for dvl_preintegrated_factor.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <boost/bind/bind.hpp>

#include "coug_fgo/factors/dvl_preintegrated_factor.hpp"

/**
 * @brief Test the error evaluation logic of the DvlPreintegratedFactor.
 *
 * Checks that the factor correctly equates the relative pose displacement between
 * nodes `i` and `j` to the preintegrated DVL measurement.
 *
 * Cases tested:
 * 1.  **Identity**: Everything aligned. Zero error.
 * 2.  **Correct Translation**: Standard translation check.
 * 3.  **Rotation + Translation**: Rotated frame check.
 * 4.  **Error Check**: Verifies non-zero error magnitude.
 */
TEST(DvlPreintegratedFactorTest, ErrorEvaluation) {
  gtsam::Key poseIKey = gtsam::symbol_shorthand::X(1);
  gtsam::Key poseJKey = gtsam::symbol_shorthand::X(2);
  gtsam::Vector3 measured_translation(1.0, 0.0, 0.0);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  coug_fgo::factors::DvlPreintegratedFactor factor(
    poseIKey, poseJKey, measured_translation, model);

  // Case 1: Identity
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3(-1, 0, 0),
      factor.evaluateError(gtsam::Pose3::Identity(), gtsam::Pose3::Identity()), 1e-9));

  // Case 2: Correct Translation
  gtsam::Pose3 pose_i = gtsam::Pose3::Identity();
  gtsam::Pose3 pose_j = gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(1, 0, 0));
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor.evaluateError(pose_i, pose_j), 1e-9));

  // Case 3: Rotation + Translation
  pose_i = gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3(0, 0, 0));
  pose_j = gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0, 1, 0));
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor.evaluateError(pose_i, pose_j), 1e-9));

  // Case 4: Error Check
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3(-1, 0, 0),
      factor.evaluateError(gtsam::Pose3::Identity(), gtsam::Pose3::Identity()), 1e-9));
}

/**
 * @brief Verify Jacobians of the DvlPreintegratedFactor using numerical differentiation.
 *
 * Validates the analytical Jacobians with respect to:
 * 1.  **Pose I**: Orientation and position affect the prediction.
 * 2.  **Pose J**: Position affects relative displacement.
 */
TEST(DvlPreintegratedFactorTest, Jacobians) {
  gtsam::Key poseIKey = gtsam::symbol_shorthand::X(1);
  gtsam::Key poseJKey = gtsam::symbol_shorthand::X(2);
  gtsam::Vector3 measured_translation(1.0, 0.5, -0.2);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  coug_fgo::factors::DvlPreintegratedFactor factor(
    poseIKey, poseJKey, measured_translation, model);

  gtsam::Pose3 pose_i = gtsam::Pose3(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1, 2, 3));
  gtsam::Pose3 pose_j = gtsam::Pose3(gtsam::Rot3::Ypr(-0.2, 0.4, 0.1), gtsam::Point3(2, 3, 2.5));

  gtsam::Matrix expectedH1 = gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3,
      gtsam::Pose3>(
    boost::bind(
      &coug_fgo::factors::DvlPreintegratedFactor::evaluateError, &factor,
      boost::placeholders::_1, boost::placeholders::_2, boost::none, boost::none),
    pose_i, pose_j, 1e-5);
  gtsam::Matrix expectedH2 = gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3,
      gtsam::Pose3>(
    boost::bind(
      &coug_fgo::factors::DvlPreintegratedFactor::evaluateError, &factor,
      boost::placeholders::_1, boost::placeholders::_2, boost::none, boost::none),
    pose_i, pose_j, 1e-5);

  gtsam::Matrix actualH1, actualH2;
  factor.evaluateError(pose_i, pose_j, actualH1, actualH2);
  EXPECT_TRUE(gtsam::assert_equal(expectedH1, actualH1, 1e-5));
  EXPECT_TRUE(gtsam::assert_equal(expectedH2, actualH2, 1e-5));
}
