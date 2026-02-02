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
 * @file constant_velocity_factor.hpp
 * @brief GTSAM factor for enforcing constant body-frame velocity.
 * @author Nelson Durrant
 * @date Feb 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo::factors
{

/**
 * @class CustomConstantVelocityFactor
 * @brief GTSAM factor for enforcing constant body velocity between two poses.
 *
 * This factor constrains the body-frame velocity of the AUV to be constant
 * between two consecutive keyframes.
 */
class CustomConstantVelocityFactor : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3,
    gtsam::Pose3, gtsam::Vector3>
{
public:
  /**
   * @brief Constructor for CustomConstantVelocityFactor.
   * @param poseKey1 GTSAM key for the first pose (state i).
   * @param velKey1 GTSAM key for the first velocity (state i).
   * @param poseKey2 GTSAM key for the second pose (state j).
   * @param velKey2 GTSAM key for the second velocity (state j).
   * @param model The noise model for the constraint.
   */
  CustomConstantVelocityFactor(
    gtsam::Key poseKey1, gtsam::Key velKey1,
    gtsam::Key poseKey2, gtsam::Key velKey2,
    const gtsam::SharedNoiseModel & model)
  : NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
      model, poseKey1, velKey1, poseKey2, velKey2) {}

  ~CustomConstantVelocityFactor() override {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose1 The first pose estimate.
   * @param vel1 The first velocity estimate.
   * @param pose2 The second pose estimate.
   * @param vel2 The second velocity estimate.
   * @param H1 Optional Jacobian matrix.
   * @param H2 Optional Jacobian matrix.
   * @param H3 Optional Jacobian matrix.
   * @param H4 Optional Jacobian matrix.
   * @return The 3D error vector.
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose1, const gtsam::Vector3 & vel1,
    const gtsam::Pose3 & pose2, const gtsam::Vector3 & vel2,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none,
    boost::optional<gtsam::Matrix &> H3 = boost::none,
    boost::optional<gtsam::Matrix &> H4 = boost::none) const override
  {
    gtsam::Rot3 R1 = pose1.rotation();
    gtsam::Rot3 R2 = pose2.rotation();

    gtsam::Matrix33 H_R1_v1, H_v1;
    gtsam::Matrix33 H_R2_v2, H_v2;

    // 3D velocity difference residual
    gtsam::Vector3 err = R1.unrotate(vel1, H1 ? &H_R1_v1 : nullptr, H2 ? &H_v1 : nullptr) -
      R2.unrotate(vel2, H3 ? &H_R2_v2 : nullptr, H4 ? &H_v2 : nullptr);

    if (H1) {
      // Jacobian with respect to pose1
      *H1 = (gtsam::Matrix36() << H_R1_v1, gtsam::Matrix33::Zero()).finished();
    }
    if (H2) {
      // Jacobian with respect to vel1
      *H2 = H_v1;
    }
    if (H3) {
      // Jacobian with respect to pose2
      *H3 = (gtsam::Matrix36() << -H_R2_v2, gtsam::Matrix33::Zero()).finished();
    }
    if (H4) {
      // Jacobian with respect to vel2
      *H4 = -H_v2;
    }

    return err;
  }
};

}  // namespace coug_fgo::factors
