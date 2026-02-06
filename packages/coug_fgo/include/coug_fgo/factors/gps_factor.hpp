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
 * @file gps_factor.hpp
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
 * @class Gps2dFactorArm
 * @brief GTSAM factor for 2D GPS position measurements with a lever arm.
 *
 * This factor constrains the 2D (horizontal) position of the AUV based on GPS,
 * accounting for the lever arm between the AUV base and the sensor.
 */
class Gps2dFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  gtsam::Point3 measured_position_;
  gtsam::Point3 base_p_sensor_;

public:
  /**
   * @brief Constructor for Gps2dFactorArm.
   * @param pose_key GTSAM key for the AUV pose.
   * @param measured_position The measured 3D position (Z is ignored).
   * @param base_T_sensor The static transform from base to sensor.
   * @param noise_model The noise model for the measurement (model dimension must be 2).
   */
  Gps2dFactorArm(
    gtsam::Key pose_key, const gtsam::Point3 & measured_position,
    const gtsam::Pose3 & base_T_sensor, const gtsam::SharedNoiseModel & noise_model)
  : NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
    measured_position_(measured_position)
  {
    base_p_sensor_ = base_T_sensor.translation();
  }

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose The AUV pose estimate.
   * @param H Optional Jacobian matrix.
   * @return The 2D error vector (measured - predicted) in [x, y].
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    // Predict the position measurement
    gtsam::Matrix36 H_full;
    gtsam::Point3 p_sensor_est = pose.transformFrom(base_p_sensor_, H ? &H_full : 0);

    // 2D position residual (ignore Z)
    gtsam::Vector2 error = (p_sensor_est - measured_position_).head<2>();

    if (H) {
      // Jacobian with respect to pose (2x6)
      *H = H_full.topRows<2>();
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
