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
 * @file mag_factor.hpp
 * @brief GTSAM factor for magnetometer yaw-only measurements with a lever arm.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo::factors
{

/**
 * @class CustomMagYawFactorArm
 * @brief GTSAM factor for magnetometer yaw-only measurements with a lever arm.
 *
 * This factor constrains the yaw orientation of the AUV based on magnetometer measurements,
 * accounting for the rotation between the AUV base and the sensor.
 */
class CustomMagYawFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  gtsam::Point3 reference_field_;
  gtsam::Rot3 base_R_sensor_;
  double measured_yaw_;

public:
  /**
   * @brief Constructor for CustomMagYawFactorArm.
   * @param pose_key GTSAM key for the AUV pose.
   * @param measured_field The measured magnetic field vector (sensor frame).
   * @param reference_field The reference magnetic field vector (world frame).
   * @param base_R_sensor The static rotation from base to sensor.
   * @param noise_model The noise model for the measurement (1D).
   */
  CustomMagYawFactorArm(
    gtsam::Key pose_key, const gtsam::Point3 & measured_field,
    const gtsam::Point3 & reference_field,
    const gtsam::Rot3 & base_R_sensor, const gtsam::SharedNoiseModel & noise_model)
  : NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
    reference_field_(reference_field),
    base_R_sensor_(base_R_sensor)
  {
    measured_yaw_ = std::atan2(measured_field.y(), measured_field.x());
  }

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose The AUV pose estimate.
   * @param H Optional Jacobian matrix.
   * @return The 1D error vector (yaw).
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    // Predict the magnetic field measurement
    gtsam::Point3 b_body = pose.rotation().unrotate(reference_field_);
    gtsam::Point3 b_sensor = base_R_sensor_.unrotate(b_body);

    double yaw_pred = std::atan2(b_sensor.y(), b_sensor.x());

    // 1D yaw residual
    double error = yaw_pred - measured_yaw_;
    while (error > M_PI) {
      error -= 2.0 * M_PI;
    }
    while (error < -M_PI) {
      error += 2.0 * M_PI;
    }

    if (H) {
      // Jacobian with respect to pose (1x6)
      H->setZero(1, 6);

      double r2 = b_sensor.x() * b_sensor.x() + b_sensor.y() * b_sensor.y();
      double inv_r2 = (r2 > 1e-9) ? (1.0 / r2) : 0.0;
      double d_yaw_dx = -b_sensor.y() * inv_r2;
      double d_yaw_dy = b_sensor.x() * inv_r2;

      gtsam::Vector3 J_inter = base_R_sensor_.rotate(gtsam::Vector3(d_yaw_dx, d_yaw_dy, 0.0));
      gtsam::Vector3 J_rot = J_inter.cross(b_body);

      (*H)(0, 0) = J_rot.x();
      (*H)(0, 1) = J_rot.y();
      (*H)(0, 2) = J_rot.z();
    }

    return gtsam::Vector1(error);
  }
};

}  // namespace coug_fgo::factors
