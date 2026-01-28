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
 * @file mag_factor_arm.hpp
 * @brief GTSAM factor for magnetometer measurements with a lever arm.
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
 * @class CustomMagFactorArm
 * @brief GTSAM factor for magnetometer measurements with a lever arm.
 *
 * This factor constrains the 3D orientation of the AUV based on magnetometer measurements,
 * accounting for the rotation between the AUV base and the sensor.
 */
class CustomMagFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  gtsam::Point3 measured_field_;
  gtsam::Point3 reference_field_;
  gtsam::Rot3 R_base_sensor_;
  bool constrain_yaw_only_;

public:
  /**
   * @brief Constructor for CustomMagFactorArm.
   * @param poseKey GTSAM key for the AUV pose.
   * @param measured_field The measured magnetic field vector (sensor frame).
   * @param reference_field The reference magnetic field vector (world frame).
   * @param R_base_sensor The static rotation from base to sensor.
   * @param model The noise model for the measurement.
   * @param constrain_yaw_only Whether to constrain only the yaw angle.
   */
  CustomMagFactorArm(
    gtsam::Key poseKey, const gtsam::Point3 & measured_field,
    const gtsam::Point3 & reference_field,
    const gtsam::Rot3 & R_base_sensor, const gtsam::SharedNoiseModel & model,
    bool constrain_yaw_only = false)
  : NoiseModelFactor1<gtsam::Pose3>(model, poseKey),
    measured_field_(measured_field),
    reference_field_(reference_field),
    R_base_sensor_(R_base_sensor),
    constrain_yaw_only_(constrain_yaw_only) {}

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
    // Predict the magnetic field in the sensor frame
    gtsam::Matrix3 H_unrotate;
    gtsam::Point3 b_body =
      pose.rotation().unrotate(reference_field_, H ? &H_unrotate : 0);
    gtsam::Point3 b_sensor = R_base_sensor_.unrotate(b_body);

    if (constrain_yaw_only_) {
      double yaw_pred = std::atan2(b_sensor.y(), b_sensor.x());
      double yaw_meas = std::atan2(measured_field_.y(), measured_field_.x());

      // 1D yaw residual
      double error = gtsam::Rot3::Ypr(yaw_meas, 0.0, 0.0).localCoordinates(
        gtsam::Rot3::Ypr(yaw_pred, 0.0, 0.0)).z();

      if (H) {
        // Jacobian with respect to pose
        double r2 = b_sensor.x() * b_sensor.x() + b_sensor.y() * b_sensor.y();
        double d_yaw_db_x = -b_sensor.y() / r2;
        double d_yaw_db_y = b_sensor.x() / r2;
        double d_yaw_db_z = 0.0;
        gtsam::Matrix13 H_atan2;
        H_atan2 << d_yaw_db_x, d_yaw_db_y, d_yaw_db_z;

        gtsam::Matrix3 H_rot = R_base_sensor_.transpose() * H_unrotate;  // d(b_sensor)/d(theta)
        gtsam::Matrix13 H_total_rot = H_atan2 * H_rot;  // Chain rule

        gtsam::Matrix13 H_pos = gtsam::Matrix13::Zero();
        *H = (gtsam::Matrix(1, 6) << H_total_rot, H_pos).finished();
      }

      return gtsam::Vector1(error);

    } else {
      // Standard 3D Residual
      if (H) {
        // Jacobian with respect to pose
        gtsam::Matrix3 H_rot = R_base_sensor_.transpose() * H_unrotate;  // d(error)/d(theta)
        gtsam::Matrix3 H_pos = gtsam::Matrix3::Zero();  // d(error)/d(pos)
        *H = (gtsam::Matrix(3, 6) << H_rot, H_pos).finished();
      }

      // 3D magnetic field residual
      return b_sensor - measured_field_;
    }
  }
};

}  // namespace coug_fgo::factors
