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
 * @file dvl_preintegrator.hpp
 * @brief Utility for preintegrating DVL velocity measurements into relative translation.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

namespace coug_fgo::utils
{

/**
 * @class DvlPreintegrator
 * @brief Utility for preintegrating DVL velocity measurements into relative translation.
 *
 * This class accumulates body-frame velocity measurements over time to compute
 * a relative translation between two timestamps, accounting for changes in orientation.
 */
class DvlPreintegrator
{
public:
  /**
   * @brief Default constructor for DvlPreintegrator.
   */
  DvlPreintegrator() {reset(gtsam::Rot3());}

  /**
   * @brief Resets the preintegrator state.
   * @param initial_orientation The orientation at the start of the integration period.
   */
  void reset(const gtsam::Rot3 & initial_orientation)
  {
    initial_orientation_ = initial_orientation;
    delta_rotation_ = gtsam::Rot3();
    integrated_translation_ = gtsam::Vector3::Zero();
    covariance_ = gtsam::Matrix3::Zero();
    dt_sum_ = 0.0;
  }

  /**
   * @brief Integrates a new DVL velocity measurement.
   * @param measured_vel The velocity measurement in the body frame.
   * @param measured_orientation The current orientation estimate.
   * @param dt The time delta since the last measurement.
   * @param measured_cov The measurement noise covariance.
   */
  void integrateMeasurement(
    const gtsam::Vector3 & measured_vel,
    const gtsam::Rot3 & measured_orientation, double dt,
    const gtsam::Matrix3 & measured_cov)
  {
    // Relative rotation from the integration start frame
    gtsam::Rot3 R_ik = initial_orientation_.inverse() * measured_orientation;
    delta_rotation_ = R_ik;

    // Accumulate the position change in the start frame
    gtsam::Vector3 vel_i = R_ik.rotate(measured_vel);
    integrated_translation_ += vel_i * dt;

    // Propagate measurement uncertainty into the covariance
    gtsam::Matrix3 J = R_ik.matrix() * dt;
    covariance_ += J * measured_cov * J.transpose();

    dt_sum_ += dt;
  }

  /**
   * @brief Gets the preintegrated translation delta.
   * @return The translation delta in the starting frame.
   */
  gtsam::Vector3 delta() const {return integrated_translation_;}

  /**
   * @brief Gets the accumulated translation covariance.
   * @return The 3x3 covariance matrix.
   */
  gtsam::Matrix3 covariance() const {return covariance_;}

private:
  gtsam::Rot3 initial_orientation_;
  gtsam::Rot3 delta_rotation_;
  gtsam::Vector3 integrated_translation_;
  gtsam::Matrix3 covariance_;
  double dt_sum_;
};

}  // namespace coug_fgo::utils
