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
 * @file hydrodynamic_factor.hpp
 * @brief GTSAM factor for enforcing a simplified version of Fossen's equations with lever arm.
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

using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo::factors
{

/**
 * @class CustomHydrodynamicDragFactorArm
 * @brief GTSAM factor for enforcing simplified hydrodynamic dynamics between two poses.
 *
 * This factor constrains the velocity evolution of the AUV based on a simplified
 * Fossen model, accounting for thruster inputs and drag.
 *
 * Model: V_next = V_curr + (dt/m) * (F_thrust - (linear_drag * V + quad_drag * |V| * V))
 */
class CustomHydrodynamicDragFactorArm : public gtsam::NoiseModelFactor4<gtsam::Pose3,
    gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>
{
private:
  double dt_;
  gtsam::Vector3 force_body_;
  double mass_;
  double linear_drag_;
  double quad_drag_;

public:
  /**
   * @brief Constructor for CustomHydrodynamicDragFactorArm.
   * @param pose_key1 GTSAM key for the first pose (state i).
   * @param vel_key1 GTSAM key for the first velocity (state i).
   * @param pose_key2 GTSAM key for the second pose (state j).
   * @param vel_key2 GTSAM key for the second velocity (state j).
   * @param dt The time interval between the two states.
   * @param control_force The sensor-frame force vector from thrusters.
   * @param body_T_sensor The transform from sensor frame to body frame.
   * @param mass Combined mass (Rigid body + Added mass).
   * @param linear_drag Linear damping coefficient.
   * @param quad_drag Quadratic damping coefficient.
   * @param noise_model The noise model for the constraint.
   */
  CustomHydrodynamicDragFactorArm(
    gtsam::Key pose_key1, gtsam::Key vel_key1,
    gtsam::Key pose_key2, gtsam::Key vel_key2,
    double dt,
    const gtsam::Vector3 & control_force,
    const gtsam::Pose3 & body_T_sensor,
    double mass,
    double linear_drag,
    double quad_drag,
    const gtsam::SharedNoiseModel & noise_model)
  : NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
      noise_model, pose_key1, vel_key1, pose_key2, vel_key2),
    dt_(dt),
    mass_(mass),
    linear_drag_(linear_drag),
    quad_drag_(quad_drag)
  {
    force_body_ = body_T_sensor.rotation() * control_force;
  }

  ~CustomHydrodynamicDragFactorArm() override {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose1 The first pose estimate.
   * @param vel1 The first velocity estimate.
   * @param pose2 The second pose estimate.
   * @param vel2 The second velocity estimate.
   * @param H1 Optional Jacobian matrix (Pose1).
   * @param H2 Optional Jacobian matrix (Vel1).
   * @param H3 Optional Jacobian matrix (Pose2).
   * @param H4 Optional Jacobian matrix (Vel2).
   * @return The 3D error vector.
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose1, const gtsam::Vector3 & vel1,
    const gtsam::Pose3 & pose2, const gtsam::Vector3 & vel2,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none,
    boost::optional<gtsam::Matrix &> H3 = boost::none,
    boost::optional<gtsam::Matrix &> H4 = boost::none) const override
  {
    // Predict the velocity measurements
    gtsam::Matrix33 J_vb1_R1, J_vb1_v1, J_vb2_R2, J_vb2_v2;
    gtsam::Vector3 v_body1 =
      pose1.rotation().unrotate(vel1, H1 ? &J_vb1_R1 : 0, H2 ? &J_vb1_v1 : 0);
    gtsam::Vector3 v_body2 =
      pose2.rotation().unrotate(vel2, H3 ? &J_vb2_R2 : 0, H4 ? &J_vb2_v2 : 0);

    gtsam::Vector3 drag_force;
    gtsam::Matrix33 J_drag_v = gtsam::Matrix33::Zero();

    for (int i = 0; i < 3; i++) {
      double u = v_body1(i);
      double abs_u = std::abs(u);

      drag_force(i) = -(linear_drag_ * u + quad_drag_ * abs_u * u);

      if (H1 || H2) {
        J_drag_v(i, i) = -(linear_drag_ + 2.0 * quad_drag_ * abs_u);
      }
    }

    gtsam::Vector3 accel_body = (force_body_ + drag_force) / mass_;
    gtsam::Vector3 v_body_pred = v_body1 + accel_body * dt_;

    // 3D velocity residual
    gtsam::Vector3 error = v_body2 - v_body_pred;

    if (H1) {
      // Jacobian with respect to pose1 (3x6)
      gtsam::Matrix33 J_scale = gtsam::Matrix33::Identity();
      J_scale.diagonal() += (dt_ / mass_) * J_drag_v.diagonal();

      H1->setZero(3, 6);
      H1->block<3, 3>(0, 0) = -J_scale * J_vb1_R1;
    }

    if (H2) {
      // Jacobian with respect to velocity1 (3x3)
      gtsam::Matrix33 J_scale = gtsam::Matrix33::Identity();
      J_scale.diagonal() += (dt_ / mass_) * J_drag_v.diagonal();

      *H2 = -J_scale * J_vb1_v1;
    }

    if (H3) {
      // Jacobian with respect to pose2 (3x6)
      H3->setZero(3, 6);
      H3->block<3, 3>(0, 0) = J_vb2_R2;
    }

    if (H4) {
      // Jacobian with respect to velocity2 (3x3)
      *H4 = J_vb2_v2;
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
