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
 * @file test_constant_velocity_factor.cpp
 * @brief Unit tests for constant_velocity_factor.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Feb 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <boost/bind/bind.hpp>

#include "coug_fgo/factors/constant_velocity_factor.hpp"

/**
 * @brief Test the error evaluation logic of the CustomConstantVelocityFactor.
 *
 * Verifies that the factor correctly constrains the body-frame velocity to be
 * constant between two poses.
 *
 * Cases tested:
 * 1.  **Identity**: Everything aligned. Zero error.
 * 2.  **Rotation**: Zero error when body velocity constant despite rotation.
 * 3.  **Error Check**: Verifies non-zero error magnitude.
 */
TEST(CustomConstantVelocityFactorTest, ErrorEvaluation) {
  gtsam::Key poseKey1 = gtsam::symbol_shorthand::X(1);
  gtsam::Key velKey1 = gtsam::symbol_shorthand::V(1);
  gtsam::Key poseKey2 = gtsam::symbol_shorthand::X(2);
  gtsam::Key velKey2 = gtsam::symbol_shorthand::V(2);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  coug_fgo::factors::CustomConstantVelocityFactor factor(
    poseKey1, velKey1, poseKey2, velKey2, model);

  // Case 1: Identity
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor.evaluateError(
        gtsam::Pose3::Identity(), gtsam::Vector3(1, 0, 0),
        gtsam::Pose3::Identity(), gtsam::Vector3(1, 0, 0)), 1e-9));

  // Case 2: Rotation
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor.evaluateError(
        gtsam::Pose3::Identity(), gtsam::Vector3(1, 0, 0),
        gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3(1, 1, 0)),
        gtsam::Vector3(0, 1, 0)), 1e-9));

  // Case 3: Error Check
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3(-1, 0, 0),
      factor.evaluateError(
        gtsam::Pose3::Identity(), gtsam::Vector3(1, 0, 0),
        gtsam::Pose3::Identity(), gtsam::Vector3(2, 0, 0)), 1e-9));
}

/**
 * @brief Verify Jacobians of the CustomConstantVelocityFactor using numerical differentiation.
 *
 * Validates the analytical Jacobians with respect to both:
 * 1.  **Pose 1**: Orientation affects projection of v1.
 * 2.  **Velocity 1**: Linear.
 * 3.  **Pose 2**: Orientation affects projection of v2.
 * 4.  **Velocity 2**: Linear.
 */
TEST(CustomConstantVelocityFactorTest, Jacobians) {
  gtsam::Key poseKey1 = gtsam::symbol_shorthand::X(1);
  gtsam::Key velKey1 = gtsam::symbol_shorthand::V(1);
  gtsam::Key poseKey2 = gtsam::symbol_shorthand::X(2);
  gtsam::Key velKey2 = gtsam::symbol_shorthand::V(2);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  coug_fgo::factors::CustomConstantVelocityFactor factor(
    poseKey1, velKey1, poseKey2, velKey2, model);

  gtsam::Pose3 pose1(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1, 2, 3));
  gtsam::Vector3 vel1(1.0, 0.5, 0.0);
  gtsam::Pose3 pose2(gtsam::Rot3::Ypr(0.4, -0.1, 0.2), gtsam::Point3(2, 3, 4));
  gtsam::Vector3 vel2(1.1, 0.4, 0.1);

  gtsam::Matrix expectedH1 = gtsam::numericalDerivative41<gtsam::Vector, gtsam::Pose3,
      gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
    boost::bind(
      &coug_fgo::factors::CustomConstantVelocityFactor::evaluateError, &factor,
      boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3,
      boost::placeholders::_4, boost::none, boost::none, boost::none, boost::none),
    pose1, vel1, pose2, vel2, 1e-5);

  gtsam::Matrix expectedH2 = gtsam::numericalDerivative42<gtsam::Vector, gtsam::Pose3,
      gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
    boost::bind(
      &coug_fgo::factors::CustomConstantVelocityFactor::evaluateError, &factor,
      boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3,
      boost::placeholders::_4, boost::none, boost::none, boost::none, boost::none),
    pose1, vel1, pose2, vel2, 1e-5);

  gtsam::Matrix expectedH3 = gtsam::numericalDerivative43<gtsam::Vector, gtsam::Pose3,
      gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
    boost::bind(
      &coug_fgo::factors::CustomConstantVelocityFactor::evaluateError, &factor,
      boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3,
      boost::placeholders::_4, boost::none, boost::none, boost::none, boost::none),
    pose1, vel1, pose2, vel2, 1e-5);

  gtsam::Matrix expectedH4 = gtsam::numericalDerivative44<gtsam::Vector, gtsam::Pose3,
      gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
    boost::bind(
      &coug_fgo::factors::CustomConstantVelocityFactor::evaluateError, &factor,
      boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3,
      boost::placeholders::_4, boost::none, boost::none, boost::none, boost::none),
    pose1, vel1, pose2, vel2, 1e-5);

  gtsam::Matrix actualH1, actualH2, actualH3, actualH4;
  factor.evaluateError(pose1, vel1, pose2, vel2, actualH1, actualH2, actualH3, actualH4);

  EXPECT_TRUE(gtsam::assert_equal(expectedH1, actualH1, 1e-5));
  EXPECT_TRUE(gtsam::assert_equal(expectedH2, actualH2, 1e-5));
  EXPECT_TRUE(gtsam::assert_equal(expectedH3, actualH3, 1e-5));
  EXPECT_TRUE(gtsam::assert_equal(expectedH4, actualH4, 1e-5));
}
