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
 * @class DvlFactor
 * @brief GTSAM factor for DVL velocity measurements in the AUV base frame.
 *
 * This factor relates the AUV's world-frame velocity and pose to the measured
 * velocity from the DVL (provided in the AUV base frame).
 */
class DvlFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>
{
  gtsam::Vector3 measured_velocity_base_;

public:
  /**
   * @brief Constructor for DvlFactor.
   * @param pose_key GTSAM key for the AUV pose.
   * @param vel_key GTSAM key for the AUV world-frame velocity.
   * @param measured_velocity_base The velocity measurement in the base frame.
   * @param noise_model The noise model for the measurement.
   */
  DvlFactor(
    gtsam::Key pose_key, gtsam::Key vel_key,
    const gtsam::Vector3 & measured_velocity_base,
    const gtsam::SharedNoiseModel & noise_model)
  : NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(noise_model, pose_key, vel_key),
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
    // Predict the velocity measurement
    gtsam::Matrix33 H_unrotate_R, H_unrotate_v;
    gtsam::Vector3 predicted_vel_base = pose.rotation().unrotate(
      vel_world, H_pose ? &H_unrotate_R : 0, H_vel ? &H_unrotate_v : 0);

    // 3D velocity residual
    gtsam::Vector3 error = predicted_vel_base - measured_velocity_base_;

    if (H_pose) {
      // Jacobian with respect to pose (3x6)
      H_pose->setZero(3, 6);
      H_pose->block<3, 3>(0, 0) = H_unrotate_R;
    }

    if (H_vel) {
      // Jacobian with respect to velocity (3x3)
      *H_vel = H_unrotate_v;
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
