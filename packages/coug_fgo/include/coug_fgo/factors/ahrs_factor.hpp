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
 * @file ahrs_factor_arm.hpp
 * @brief GTSAM factor for AHRS/orientation yaw-only measurements with a lever arm.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo::factors
{

/**
 * @class AhrsYawFactorArm
 * @brief GTSAM factor for AHRS/orientation yaw-only measurements with a lever arm.
 *
 * This factor constrains the yaw orientation of the AUV based on AHRS/IMU measurements,
 * accounting for the rotation between the AUV base and the sensor.
 */
class AhrsYawFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  gtsam::Rot3 measured_rot_sensor_;
  gtsam::Rot3 base_R_sensor_;
  double measured_yaw_base_;

public:
  /**
   * @brief Constructor for AhrsYawFactorArm.
   * @param pose_key GTSAM key for the AUV pose.
   * @param measured_rot_sensor The measured orientation of the sensor in the world frame.
   * @param base_R_sensor The static rotation from base to sensor.
   * @param mag_declination Magnetic declination to add to the measurement [rad].
   * @param noise_model The noise model for the measurement (1D).
   */
  AhrsYawFactorArm(
    gtsam::Key pose_key, const gtsam::Rot3 & measured_rot_sensor,
    const gtsam::Rot3 & base_R_sensor, double mag_declination,
    const gtsam::SharedNoiseModel & noise_model)
  : NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
    measured_rot_sensor_(measured_rot_sensor),
    base_R_sensor_(base_R_sensor)
  {
    gtsam::Rot3 R_decl = gtsam::Rot3::Yaw(mag_declination);
    gtsam::Rot3 measured_rot_base = (R_decl * measured_rot_sensor) * base_R_sensor.inverse();
    measured_yaw_base_ = measured_rot_base.yaw();
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
    // Predict the yaw measurement
    const gtsam::Rot3 & R_est_base = pose.rotation();
    double yaw_est = R_est_base.yaw();

    // 1D yaw residual
    double error = yaw_est - measured_yaw_base_;
    while (error > M_PI) {
      error -= 2.0 * M_PI;
    }
    while (error < -M_PI) {
      error += 2.0 * M_PI;
    }

    if (H) {
      // Jacobian with respect to pose (1x6)
      H->setZero(1, 6);

      const gtsam::Matrix33 & R = R_est_base.matrix();
      double R00 = R(0, 0);
      double R10 = R(1, 0);

      double D = R00 * R00 + R10 * R10;
      if (D > 1e-9) {
        (*H)(0, 1) = (R10 * R(0, 2) - R00 * R(1, 2)) / D;
        (*H)(0, 2) = (R00 * R(1, 1) - R10 * R(0, 1)) / D;
      } else {
        (*H)(0, 2) = 1.0;
      }
    }

    return gtsam::Vector1(error);
  }
};

}  // namespace coug_fgo::factors
