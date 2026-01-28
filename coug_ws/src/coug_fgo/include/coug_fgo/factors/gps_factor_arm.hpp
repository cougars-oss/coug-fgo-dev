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
 * @file gps_factor_arm.hpp
 * @brief GTSAM factor for GPS position measurements with a lever arm.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo::factors
{

/**
 * @class CustomGPSFactorArm
 * @brief GTSAM factor for GPS position measurements with a lever arm.
 *
 * This factor constrains the 3D position of the AUV based on GPS measurements,
 * accounting for the lever arm between the AUV base and the sensor.
 */
class CustomGPSFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  gtsam::Point3 measured_position_;
  gtsam::Pose3 T_base_sensor_;

public:
  /**
   * @brief Constructor for CustomGPSFactorArm.
   * @param poseKey GTSAM key for the AUV pose.
   * @param measured_position The measured 3D position in the world frame.
   * @param T_base_sensor The static transform from base to sensor.
   * @param model The noise model for the measurement.
   */
  CustomGPSFactorArm(
    gtsam::Key poseKey, const gtsam::Point3 & measured_position,
    const gtsam::Pose3 & T_base_sensor, const gtsam::SharedNoiseModel & model)
  : NoiseModelFactor1<gtsam::Pose3>(model, poseKey),
    measured_position_(measured_position),
    T_base_sensor_(T_base_sensor) {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose The AUV pose estimate.
   * @param H Optional Jacobian matrix.
   * @return The 3D error vector (measured - predicted).
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    gtsam::Pose3 T_ws = pose.compose(T_base_sensor_);

    // 3D position residual
    gtsam::Vector3 error = T_ws.translation() - measured_position_;

    if (H) {
      // Jacobian with respect to pose
      gtsam::Matrix H_matrix = gtsam::Matrix::Zero(3, 6);

      gtsam::Matrix R_wb = pose.rotation().matrix();
      gtsam::Vector3 p_bs = T_base_sensor_.translation();
      gtsam::Matrix p_bs_skew = gtsam::skewSymmetric(p_bs);

      H_matrix.block<3, 3>(0, 0) = -R_wb * p_bs_skew;  // d(error)/d(theta)
      H_matrix.block<3, 3>(0, 3) = R_wb;  // d(error)/d(pos)

      *H = H_matrix;
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
