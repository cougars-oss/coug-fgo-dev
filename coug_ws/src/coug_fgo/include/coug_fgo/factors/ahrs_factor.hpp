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
 * @file ahrs_factor.hpp
 * @brief GTSAM factor for AHRS/orientation measurements with a lever arm.
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
 * @class AhrsFactor
 * @brief GTSAM factor for AHRS/orientation measurements with a lever arm.
 *
 * This factor constrains the 3D orientation of the AUV based on AHRS/IMU measurements,
 * accounting for the rotation between the AUV base and the sensor.
 */
class CustomAHRSFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  gtsam::Rot3 measured_rot_sensor_;
  gtsam::Rot3 R_base_sensor_;
  double mag_declination_;

public:
  /**
   * @brief Constructor for CustomAHRSFactor.
   * @param poseKey GTSAM key for the AUV pose.
   * @param measured_rot_sensor The measured orientation of the sensor in the world frame.
   * @param R_base_sensor The static rotation from base to sensor.
   * @param mag_declination Magnetic declination to add to the measurement [rad].
   * @param model The noise model for the measurement.
   */
  CustomAHRSFactor(
    gtsam::Key poseKey, const gtsam::Rot3 & measured_rot_sensor,
    const gtsam::Rot3 & R_base_sensor, double mag_declination,
    const gtsam::SharedNoiseModel & model)
  : NoiseModelFactor1<gtsam::Pose3>(model, poseKey),
    measured_rot_sensor_(measured_rot_sensor),
    R_base_sensor_(R_base_sensor),
    mag_declination_(mag_declination) {}

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
    gtsam::Rot3 R_decl = gtsam::Rot3::Yaw(mag_declination_);
    gtsam::Rot3 measured_rot_sensor_true = R_decl * measured_rot_sensor_;

    gtsam::Rot3 measured_rot_base = measured_rot_sensor_true * R_base_sensor_.inverse();
    gtsam::Rot3 error_rot = measured_rot_base.inverse() * pose.rotation();

    // 3D orientation residual
    gtsam::Vector3 error = gtsam::Rot3::Logmap(error_rot);

    if (H) {
      // Jacobian with respect to pose
      gtsam::Matrix3 H_rot = gtsam::Rot3::LogmapDerivative(error);  // d(error)/d(theta)
      gtsam::Matrix3 H_trans = gtsam::Matrix3::Zero();  // d(error)/d(pos)
      *H = (gtsam::Matrix(3, 6) << H_rot, H_trans).finished();
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
