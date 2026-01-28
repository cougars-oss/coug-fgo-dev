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
 * @file depth_factor_arm.hpp
 * @brief GTSAM factor for depth (Z) measurements with a lever arm.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo::factors
{

/**
 * @class CustomDepthFactorArm
 * @brief GTSAM factor for depth (Z) measurements with a lever arm.
 *
 * This factor constrains the Z position of the AUV based on depth sensor measurements,
 * accounting for the lever arm between the AUV base and the sensor.
 */
class CustomDepthFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  double measured_depth_;
  gtsam::Pose3 T_base_sensor_;

public:
  /**
   * @brief Constructor for CustomDepthFactorArm.
   * @param poseKey GTSAM key for the AUV pose.
   * @param measured_depth The depth measurement (Z-axis).
   * @param T_base_sensor The static transform from base to sensor.
   * @param model The noise model for the measurement.
   */
  CustomDepthFactorArm(
    gtsam::Key poseKey, double measured_depth,
    const gtsam::Pose3 & T_base_sensor, const gtsam::SharedNoiseModel & model)
  : NoiseModelFactor1<gtsam::Pose3>(model, poseKey),
    measured_depth_(measured_depth),
    T_base_sensor_(T_base_sensor) {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose The AUV pose estimate.
   * @param H Optional Jacobian matrix.
   * @return The 1D error vector (measured - predicted).
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    gtsam::Pose3 T_ws = pose.compose(T_base_sensor_);

    // 1D Z-axis residual
    double error = T_ws.z() - measured_depth_;

    if (H) {
      // Jacobian with respect to pose
      gtsam::Matrix H_matrix = gtsam::Matrix::Zero(1, 6);

      gtsam::Matrix R_wb = pose.rotation().matrix();
      gtsam::Vector3 p_bs = T_base_sensor_.translation();
      gtsam::Matrix p_bs_skew = gtsam::skewSymmetric(p_bs);
      gtsam::Matrix R_row2 = R_wb.row(2);

      H_matrix.block<1, 3>(0, 0) = -R_row2 * p_bs_skew;  // d(error)/d(theta)
      H_matrix.block<1, 3>(0, 3) = R_row2;  // d(error)/d(pos)

      *H = H_matrix;
    }

    return gtsam::Vector1(error);
  }
};

}  // namespace coug_fgo::factors
