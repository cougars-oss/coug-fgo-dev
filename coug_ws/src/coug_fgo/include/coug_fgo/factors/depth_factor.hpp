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
 * @file depth_factor.hpp
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
  gtsam::Point3 base_p_sensor_;

public:
  /**
   * @brief Constructor for CustomDepthFactorArm.
   * @param pose_key GTSAM key for the AUV pose.
   * @param measured_depth The depth measurement (Z-axis).
   * @param base_T_sensor The static transform from base to sensor.
   * @param noise_model The noise model for the measurement.
   */
  CustomDepthFactorArm(
    gtsam::Key pose_key, double measured_depth,
    const gtsam::Pose3 & base_T_sensor, const gtsam::SharedNoiseModel & noise_model)
  : NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
    measured_depth_(measured_depth)
  {
    base_p_sensor_ = base_T_sensor.translation();
  }

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
    // Predict the depth measurement
    gtsam::Matrix36 H_full;
    gtsam::Point3 p_sensor_est = pose.transformFrom(base_p_sensor_, H ? &H_full : 0);

    // 1D depth residual
    double error = p_sensor_est.z() - measured_depth_;

    if (H) {
      // Jacobian with respect to pose (1x6)
      *H = H_full.row(2);
    }

    return gtsam::Vector1(error);
  }
};

}  // namespace coug_fgo::factors
