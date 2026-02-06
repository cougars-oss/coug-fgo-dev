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
 * @file dvl_preintegrated_factor.hpp
 * @brief GTSAM factor for preintegrated DVL translation measurements.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo::factors
{

/**
 * @class DvlPreintegratedFactor
 * @brief GTSAM factor for preintegrated DVL translation measurements.
 *
 * This factor relates two AUV poses based on a
 * preintegrated translation measurement derived from DVL data.
 */
class DvlPreintegratedFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
  gtsam::Vector3 measured_translation_;

public:
  /**
   * @brief Constructor for DvlPreintegratedFactor.
   * @param pose_key_i GTSAM key for the starting AUV pose.
   * @param pose_key_j GTSAM key for the ending AUV pose.
   * @param measured_translation The preintegrated translation measurement.
   * @param noise_model The noise model for the measurement.
   */
  DvlPreintegratedFactor(
    gtsam::Key pose_key_i, gtsam::Key pose_key_j,
    const gtsam::Vector3 & measured_translation,
    gtsam::SharedNoiseModel noise_model)
  : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(noise_model, pose_key_i, pose_key_j),
    measured_translation_(measured_translation) {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose_i Starting AUV pose estimate.
   * @param pose_j Ending AUV pose estimate.
   * @param H_pose_i Optional Jacobian matrix with respect to pose_i.
   * @param H_pose_j Optional Jacobian matrix with respect to pose_j.
   * @return The 3D error vector (measured - predicted).
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose_i,
    const gtsam::Pose3 & pose_j,
    boost::optional<gtsam::Matrix &> H_pose_i = boost::none,
    boost::optional<gtsam::Matrix &> H_pose_j = boost::none) const override
  {
    // Predict the translation measurement
    gtsam::Matrix36 H_tj_posej;
    gtsam::Point3 t_j = pose_j.translation(H_pose_j ? &H_tj_posej : 0);

    gtsam::Matrix36 H_pij_posei;
    gtsam::Matrix33 H_pij_tj;
    gtsam::Point3 p_ij = pose_i.transformTo(
      t_j, H_pose_i ? &H_pij_posei : 0,
      H_pose_j ? &H_pij_tj : 0);

    // 3D translation residual
    gtsam::Vector3 error = p_ij - measured_translation_;

    if (H_pose_i) {
      // Jacobian with respect to starting pose (3x6)
      *H_pose_i = H_pij_posei;
    }

    if (H_pose_j) {
      // Jacobian with respect to ending pose (3x6)
      *H_pose_j = H_pij_tj * H_tj_posej;
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
