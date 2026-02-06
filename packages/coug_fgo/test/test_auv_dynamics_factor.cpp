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
 * @file test_auv_dynamics_factor.cpp
 * @brief Unit tests for auv_dynamics_factor.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <boost/bind/bind.hpp>

#include "coug_fgo/factors/auv_dynamics_factor.hpp"

/**
 * @brief Test the error evaluation logic of the AuvDynamicsFactorArm.
 *
 * Verifies that the factor correctly implements the Fossen dynamics model.
 * V_next = V_curr + (dt/m) * (F_thrust - (linear_drag * V + quad_drag * |V| * V))
 *
 * Cases tested:
 * 1.  **Identity**: Everything aligned. Zero error.
 * 2.  **Dynamic Identity**: Non-zero interactions (Thrust/Drag) satisfying physics.
 * 3.  **Rotation**: Vehicle rotated, sensor aligned.
 * 4.  **Mounting/Lever Arm**: Sensor offset relative to body.
 * 5.  **Combined**: Vehicle rotated + Sensor offset.
 * 6.  **Error Check**: Verifies non-zero error magnitude.
 */
TEST(AuvDynamicsFactorArmTest, ZeroError) {
  gtsam::Key poseKey1 = gtsam::symbol_shorthand::X(1);
  gtsam::Key velKey1 = gtsam::symbol_shorthand::V(1);
  gtsam::Key poseKey2 = gtsam::symbol_shorthand::X(2);
  gtsam::Key velKey2 = gtsam::symbol_shorthand::V(2);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  double dt = 1.0;
  gtsam::Matrix33 mass = gtsam::Matrix33::Identity() * 10.0;
  gtsam::Matrix33 linear_drag = gtsam::Matrix33::Zero();
  gtsam::Matrix33 quad_drag = gtsam::Matrix33::Zero();
  gtsam::Vector3 control_force = gtsam::Vector3::Zero();
  gtsam::Pose3 body_P_sensor = gtsam::Pose3::Identity();

  // Case 1: Identity
  coug_fgo::factors::AuvDynamicsFactorArm factor(
    poseKey1, velKey1, poseKey2, velKey2, dt, control_force, body_P_sensor,
    mass, linear_drag, quad_drag, model);

  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor.evaluateError(
        gtsam::Pose3::Identity(), gtsam::Vector3::Zero(),
        gtsam::Pose3::Identity(), gtsam::Vector3::Zero()), 1e-9));

  // Case 2: Dynamic Identity
  // Subcase A: Constant Acceleration
  control_force = gtsam::Vector3(10.0, 0.0, 0.0);
  coug_fgo::factors::AuvDynamicsFactorArm factor_thrust(
    poseKey1, velKey1, poseKey2, velKey2, dt, control_force, body_P_sensor,
    mass, linear_drag, quad_drag, model);

  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor_thrust.evaluateError(
        gtsam::Pose3::Identity(), gtsam::Vector3::Zero(),
        gtsam::Pose3::Identity(), gtsam::Vector3(1.0, 0.0, 0.0)), 1e-9));

  // Subcase B: Drag Equilibrium
  linear_drag = gtsam::Matrix33::Identity() * 1.0;
  control_force = gtsam::Vector3(10.0, 0.0, 0.0);
  coug_fgo::factors::AuvDynamicsFactorArm factor_drag(
    poseKey1, velKey1, poseKey2, velKey2, dt, control_force, body_P_sensor,
    mass, linear_drag, quad_drag, model);

  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor_drag.evaluateError(
        gtsam::Pose3::Identity(), gtsam::Vector3(10.0, 0.0, 0.0),
        gtsam::Pose3::Identity(), gtsam::Vector3(10.0, 0.0, 0.0)), 1e-9));

  // Case 3: Rotation
  linear_drag = gtsam::Matrix33::Zero();
  control_force = gtsam::Vector3(10.0, 0.0, 0.0);
  coug_fgo::factors::AuvDynamicsFactorArm factor_rot(
    poseKey1, velKey1, poseKey2, velKey2, dt, control_force, gtsam::Pose3::Identity(),
    mass, linear_drag, quad_drag, model);

  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor_rot.evaluateError(
        gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3::Zero()),
        gtsam::Vector3::Zero(),
        gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3::Zero()),
        gtsam::Vector3(0.0, 1.0, 0.0)), 1e-9));


  // Case 4: Mounting/Lever Arm
  body_P_sensor = gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3::Zero());
  control_force = gtsam::Vector3(10.0, 0.0, 0.0);
  coug_fgo::factors::AuvDynamicsFactorArm factor_mount(
    poseKey1, velKey1, poseKey2, velKey2, dt, control_force, body_P_sensor,
    mass, linear_drag, quad_drag, model);

  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor_mount.evaluateError(
        gtsam::Pose3::Identity(), gtsam::Vector3::Zero(),
        gtsam::Pose3::Identity(), gtsam::Vector3(0.0, 1.0, 0.0)), 1e-9));

  // Case 5: Combined
  gtsam::Pose3 body_pose = gtsam::Pose3(gtsam::Rot3::Roll(M_PI_2), gtsam::Point3::Zero());
  coug_fgo::factors::AuvDynamicsFactorArm factor_comb(
    poseKey1, velKey1, poseKey2, velKey2, dt, control_force, body_P_sensor,
    mass, linear_drag, quad_drag, model);

  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor_comb.evaluateError(
        body_pose, gtsam::Vector3::Zero(),
        body_pose, gtsam::Vector3(0.0, 0.0, 1.0)), 1e-9));

  // Case 6: Error Check
  coug_fgo::factors::AuvDynamicsFactorArm factor_err(
    poseKey1, velKey1, poseKey2, velKey2, dt, gtsam::Vector3::Zero(), gtsam::Pose3::Identity(),
    mass, gtsam::Matrix33::Zero(), gtsam::Matrix33::Zero(), model);

  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3(1.0, 0.0, 0.0),
      factor_err.evaluateError(
        gtsam::Pose3::Identity(), gtsam::Vector3::Zero(),
        gtsam::Pose3::Identity(), gtsam::Vector3(1.0, 0.0, 0.0)), 1e-9));
}

/**
 * @brief Verify Jacobians of the AuvDynamicsFactorArm using numerical differentiation.
 *
 * Validates the analytical Jacobians with respect to:
 * 1.  **Pose 1**: Orientation affects projection of v1.
 * 2.  **Velocity 1**: Nonlinear drag dependence.
 * 3.  **Pose 2**: Orientation affects projection of v2.
 * 4.  **Velocity 2**: Linear.
 */
TEST(AuvDynamicsFactorArmTest, Jacobians) {
  gtsam::Key poseKey1 = gtsam::symbol_shorthand::X(1);
  gtsam::Key velKey1 = gtsam::symbol_shorthand::V(1);
  gtsam::Key poseKey2 = gtsam::symbol_shorthand::X(2);
  gtsam::Key velKey2 = gtsam::symbol_shorthand::V(2);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  double dt = 0.5;
  gtsam::Matrix33 mass = gtsam::Matrix33::Identity() * 5.0;
  gtsam::Matrix33 linear_drag = gtsam::Matrix33::Identity() * 1.0;
  gtsam::Matrix33 quad_drag = gtsam::Matrix33::Identity() * 0.5;
  gtsam::Vector3 control_force(2.0, -1.0, 0.5);
  gtsam::Pose3 body_P_sensor = gtsam::Pose3(gtsam::Rot3::Roll(0.1), gtsam::Point3(0.1, 0, 0));

  coug_fgo::factors::AuvDynamicsFactorArm factor(
    poseKey1, velKey1, poseKey2, velKey2, dt, control_force, body_P_sensor,
    mass, linear_drag, quad_drag, model);

  gtsam::Pose3 pose1(gtsam::Rot3::Ypr(0.1, 0.2, 0.3), gtsam::Point3(1, 2, 3));
  gtsam::Vector3 vel1(1.0, -0.5, 0.2);
  gtsam::Pose3 pose2(gtsam::Rot3::Ypr(0.4, -0.1, 0.2), gtsam::Point3(2, 3, 4));
  gtsam::Vector3 vel2(1.1, -0.4, 0.25);

  gtsam::Matrix expectedH1 = gtsam::numericalDerivative41<gtsam::Vector, gtsam::Pose3,
      gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
    boost::bind(
      &coug_fgo::factors::AuvDynamicsFactorArm::evaluateError, &factor,
      boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3,
      boost::placeholders::_4, boost::none, boost::none, boost::none, boost::none),
    pose1, vel1, pose2, vel2, 1e-5);

  gtsam::Matrix expectedH2 = gtsam::numericalDerivative42<gtsam::Vector, gtsam::Pose3,
      gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
    boost::bind(
      &coug_fgo::factors::AuvDynamicsFactorArm::evaluateError, &factor,
      boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3,
      boost::placeholders::_4, boost::none, boost::none, boost::none, boost::none),
    pose1, vel1, pose2, vel2, 1e-5);

  gtsam::Matrix expectedH3 = gtsam::numericalDerivative43<gtsam::Vector, gtsam::Pose3,
      gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
    boost::bind(
      &coug_fgo::factors::AuvDynamicsFactorArm::evaluateError, &factor,
      boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3,
      boost::placeholders::_4, boost::none, boost::none, boost::none, boost::none),
    pose1, vel1, pose2, vel2, 1e-5);

  gtsam::Matrix expectedH4 = gtsam::numericalDerivative44<gtsam::Vector, gtsam::Pose3,
      gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
    boost::bind(
      &coug_fgo::factors::AuvDynamicsFactorArm::evaluateError, &factor,
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
