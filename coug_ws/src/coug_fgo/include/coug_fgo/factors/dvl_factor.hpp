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
 * @file dvl_factor.hpp
 * @brief GTSAM factor for DVL velocity measurements in the AUV base frame.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using gtsam::symbol_shorthand::B;  // Bias (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo::factors
{

/**
 * @class CustomDVLFactor
 * @brief GTSAM factor for DVL velocity measurements in the AUV base frame.
 *
 * This factor relates the AUV's world-frame velocity and pose to the measured
 * velocity from the DVL (provided in the AUV base frame).
 */
class CustomDVLFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>
{
  gtsam::Vector3 measured_velocity_base_;

public:
  /**
   * @brief Constructor for CustomDVLFactor.
   * @param poseKey GTSAM key for the AUV pose.
   * @param velKey GTSAM key for the AUV world-frame velocity.
   * @param measured_velocity_base The velocity measurement in the base frame.
   * @param model The noise model for the measurement.
   */
  CustomDVLFactor(
    gtsam::Key poseKey, gtsam::Key velKey,
    const gtsam::Vector3 & measured_velocity_base,
    const gtsam::SharedNoiseModel & model)
  : NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(model, poseKey, velKey),
    measured_velocity_base_(measured_velocity_base) {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose The AUV pose estimate.
   * @param vel_world The AUV world-frame velocity estimate.
   * @param H_pose Optional Jacobian matrix with respect to pose.
   * @param H_vel Optional Jacobian matrix with respect to velocity.
   * @return The 3D error vector (measured - predicted).
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    const gtsam::Vector3 & vel_world,
    boost::optional<gtsam::Matrix &> H_pose = boost::none,
    boost::optional<gtsam::Matrix &> H_vel = boost::none) const override
  {
    gtsam::Rot3 R_bw = pose.rotation().inverse();
    gtsam::Vector3 predicted_vel_base = R_bw.rotate(vel_world);

    // 3D velocity residual
    gtsam::Vector3 error = predicted_vel_base - measured_velocity_base_;

    if (H_pose) {
      // Jacobian with respect to pose
      gtsam::Matrix3 H_rot = gtsam::skewSymmetric(predicted_vel_base);  // d(error)/d(theta)
      gtsam::Matrix3 H_trans = gtsam::Matrix3::Zero();  // d(error)/d(pos)
      *H_pose = (gtsam::Matrix(3, 6) << H_rot, H_trans).finished();
    }

    if (H_vel) {
      // Jacobian with respect to velocity
      *H_vel = R_bw.matrix();
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
