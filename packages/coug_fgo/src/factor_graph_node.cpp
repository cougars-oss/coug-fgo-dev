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
 * @file factor_graph_node.cpp
 * @brief Implementation of the FactorGraphNode.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#include "coug_fgo/factor_graph_node.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <chrono>

#include <rclcpp_components/register_node_macro.hpp>

#include "coug_fgo/factors/depth_factor.hpp"
#include "coug_fgo/factors/dvl_factor.hpp"
#include "coug_fgo/factors/dvl_preintegrated_factor.hpp"
#include "coug_fgo/factors/gps_factor.hpp"
#include "coug_fgo/factors/ahrs_factor.hpp"
#include "coug_fgo/factors/mag_factor.hpp"
#include "coug_fgo/factors/auv_dynamics_factor.hpp"
#include "coug_fgo/factors/constant_velocity_factor.hpp"
#include "coug_fgo/utils/conversion_utils.hpp"


using coug_fgo::factors::DepthFactorArm;
using coug_fgo::factors::DvlFactor;
using coug_fgo::factors::DvlPreintegratedFactor;
using coug_fgo::factors::Gps2dFactorArm;
using coug_fgo::factors::AhrsYawFactorArm;
using coug_fgo::factors::MagFactorArm;
using coug_fgo::factors::AuvDynamicsFactorArm;
using coug_fgo::factors::ConstantVelocityFactor;
using coug_fgo::utils::toGtsam;
using coug_fgo::utils::toGtsam3x3;
using coug_fgo::utils::toGtsamDiagonal;
using coug_fgo::utils::toGtsamSquaredDiagonal;
using coug_fgo::utils::toQuatMsg;
using coug_fgo::utils::toPointMsg;
using coug_fgo::utils::toVectorMsg;
using coug_fgo::utils::toPoseMsg;
using coug_fgo::utils::toCovariance36Msg;
using coug_fgo::utils::toPoseCovarianceMsg;
using coug_fgo::utils::DvlPreintegrator;

using gtsam::symbol_shorthand::B;  // Bias (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo
{

void FactorGraphNode::setupRosInterfaces()
{
  // --- ROS TF Interfaces ---
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // --- ROS Publishers ---
  global_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(params_.global_odom_topic, 10);
  smoothed_path_pub_ = create_publisher<nav_msgs::msg::Path>(params_.smoothed_path_topic, 10);
  velocity_pub_ =
    create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(params_.velocity_topic, 10);
  imu_bias_pub_ =
    create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(params_.imu_bias_topic, 10);
  graph_metrics_pub_ =
    create_publisher<coug_fgo_msgs::msg::GraphMetrics>(params_.graph_metrics_topic, 10);

  // --- ROS Callback Groups ---
  sensor_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sensor_options = rclcpp::SubscriptionOptions();
  sensor_options.callback_group = sensor_cb_group_;

  // --- ROS Subscribers ---
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    params_.imu_topic, 200,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      if (!have_imu_to_dvl_tf_) {
        try {
          std::string parent = params_.dvl.use_parameter_frame ?
          params_.dvl.parameter_frame : dvl_frame_;
          std::string child = params_.imu.use_parameter_frame ?
          params_.imu.parameter_frame : msg->header.frame_id;
          if (!parent.empty()) {
            imu_to_dvl_tf_ = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);
            have_imu_to_dvl_tf_ = true;
            imu_frame_ = child;
          }
        } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(get_logger(), "Failed to lookup IMU to DVL transform: %s", ex.what());
        }
      }
      imu_queue_.push(msg);
    },
    sensor_options);

  gps_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    params_.gps_odom_topic, 20,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      if ((params_.gps.enable_gps || params_.gps.enable_gps_init_only) && !have_gps_to_dvl_tf_) {
        try {
          std::string parent = params_.dvl.use_parameter_frame ?
          params_.dvl.parameter_frame : dvl_frame_;
          std::string child = params_.gps.use_parameter_frame ?
          params_.gps.parameter_frame : msg->child_frame_id;
          if (!parent.empty()) {
            gps_to_dvl_tf_ = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);
            have_gps_to_dvl_tf_ = true;
          }
        } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(get_logger(), "Failed to lookup GPS to DVL transform: %s", ex.what());
        }
      }
      gps_queue_.push(msg);
    },
    sensor_options);

  depth_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    params_.depth_odom_topic, 20,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      if (!have_depth_to_dvl_tf_) {
        try {
          std::string parent = params_.dvl.use_parameter_frame ?
          params_.dvl.parameter_frame : dvl_frame_;
          std::string child = params_.depth.use_parameter_frame ?
          params_.depth.parameter_frame : msg->child_frame_id;
          if (!parent.empty()) {
            depth_to_dvl_tf_ = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);
            have_depth_to_dvl_tf_ = true;
          }
        } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(get_logger(), "Failed to lookup depth to DVL transform: %s", ex.what());
        }
      }
      depth_queue_.push(msg);

      double time_since_dvl = this->get_clock()->now().seconds() - dvl_queue_.getLastTime();
      bool dvl_timed_out = time_since_dvl > params_.dvl.timeout_threshold;

      if (params_.experimental.enable_dvl_preintegration) {
        if (state_ != State::RUNNING) {
          initializeGraph();
        } else {
          optimizeGraph();
        }
      } else {
        if (state_ == State::RUNNING && dvl_timed_out) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "DVL timed out (%.2fs)! Using depth sensor to trigger keyframes.",
            time_since_dvl);
          optimizeGraph();
        }
      }
    },
    sensor_options);

  mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
    params_.mag_topic, 20,
    [this](const sensor_msgs::msg::MagneticField::SharedPtr msg) {
      if ((params_.mag.enable_mag || params_.mag.enable_mag_init_only) && !have_mag_to_dvl_tf_) {
        try {
          std::string parent = params_.dvl.use_parameter_frame ?
          params_.dvl.parameter_frame : dvl_frame_;
          std::string child = params_.mag.use_parameter_frame ?
          params_.mag.parameter_frame : msg->header.frame_id;
          if (!parent.empty()) {
            mag_to_dvl_tf_ = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);
            have_mag_to_dvl_tf_ = true;
          }
        } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(get_logger(), "Failed to lookup mag to DVL transform: %s", ex.what());
        }
      }
      mag_queue_.push(msg);
    },
    sensor_options);

  ahrs_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    params_.ahrs_topic, 20,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      if ((params_.ahrs.enable_ahrs || params_.ahrs.enable_ahrs_init_only) &&
      !have_ahrs_to_dvl_tf_)
      {
        try {
          std::string parent = params_.dvl.use_parameter_frame ?
          params_.dvl.parameter_frame : dvl_frame_;
          std::string child = params_.ahrs.use_parameter_frame ?
          params_.ahrs.parameter_frame : msg->header.frame_id;
          if (!parent.empty()) {
            ahrs_to_dvl_tf_ = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);
            have_ahrs_to_dvl_tf_ = true;
          }
        } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(get_logger(), "Failed to lookup AHRS to DVL transform: %s", ex.what());
        }
      }
      ahrs_queue_.push(msg);
    },
    sensor_options);

  dvl_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    params_.dvl_topic, 20,
    [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
      if (!have_dvl_to_base_tf_) {
        try {
          std::string frame = params_.dvl.use_parameter_frame ?
          params_.dvl.parameter_frame : msg->header.frame_id;
          dvl_to_base_tf_ = tf_buffer_->lookupTransform(
            params_.base_frame, frame,
            tf2::TimePointZero);
          have_dvl_to_base_tf_ = true;
          dvl_frame_ = frame;
        } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(get_logger(), "Failed to lookup DVL to base transform: %s", ex.what());
        }
      }
      dvl_queue_.push(msg);

      if (!params_.experimental.enable_dvl_preintegration) {
        if (state_ != State::RUNNING) {
          initializeGraph();
        } else {
          optimizeGraph();
        }
      }
    },
    sensor_options);

  wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    params_.wrench_topic, 20,
    [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
      if (params_.dynamics.enable_dynamics && !have_com_to_dvl_tf_) {
        try {
          std::string parent = params_.dvl.use_parameter_frame ?
          params_.dvl.parameter_frame : dvl_frame_;
          std::string child = params_.dynamics.use_parameter_frame ?
          params_.dynamics.parameter_frame : msg->header.frame_id;
          if (!parent.empty()) {
            com_to_dvl_tf_ = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);
            have_com_to_dvl_tf_ = true;
          }
        } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(get_logger(), "Failed to lookup COM to DVL transform: %s", ex.what());
        }
      }
      wrench_queue_.push(msg);
    },
    sensor_options);

  // --- ROS Diagnostics ---
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/factor_graph_node");

    std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";
    std::string suffix = params_.experimental.enable_dvl_preintegration ? " (TM)" : "";

    std::string sensor_task = prefix + "Sensor Inputs" + suffix;
    diagnostic_updater_.add(sensor_task, this, &FactorGraphNode::checkSensorInputs);

    std::string state_task = prefix + "Graph State" + suffix;
    diagnostic_updater_.add(state_task, this, &FactorGraphNode::checkGraphState);

    std::string overflow_task = prefix + "Processing Overflow" + suffix;
    diagnostic_updater_.add(overflow_task, this, &FactorGraphNode::checkProcessingOverflow);
  }
}

FactorGraphNode::FactorGraphNode(const rclcpp::NodeOptions & options)
: Node("factor_graph_node", options),
  diagnostic_updater_(this)
{
  RCLCPP_INFO(get_logger(), "Starting Factor Graph Node...");

  param_listener_ = std::make_shared<factor_graph_node::ParamListener>(
    get_node_parameters_interface());
  params_ = param_listener_->get_params();

  setupRosInterfaces();
  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for sensor messages...");
}

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
FactorGraphNode::configureImuPreintegration()
{
  auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
  imu_params->n_gravity =
    gtsam::Vector3(params_.imu.gravity[0], params_.imu.gravity[1], params_.imu.gravity[2]);
  imu_params->body_P_sensor = toGtsam(imu_to_dvl_tf_.transform);

  if (params_.imu.use_parameter_covariance) {
    imu_params->accelerometerCovariance = toGtsamSquaredDiagonal(
      params_.imu.parameter_covariance.accel_noise_sigmas).block<3, 3>(0, 0);
  } else {
    imu_params->accelerometerCovariance = toGtsam(initial_imu_->linear_acceleration_covariance);
  }

  if (params_.imu.use_parameter_covariance) {
    imu_params->gyroscopeCovariance = toGtsamSquaredDiagonal(
      params_.imu.parameter_covariance.gyro_noise_sigmas).block<3, 3>(0, 0);
  } else {
    imu_params->gyroscopeCovariance = toGtsam(initial_imu_->angular_velocity_covariance);
  }
  imu_params->biasAccCovariance = toGtsamSquaredDiagonal(params_.imu.accel_bias_rw_sigmas);
  imu_params->biasOmegaCovariance = toGtsamSquaredDiagonal(params_.imu.gyro_bias_rw_sigmas);
  imu_params->integrationCovariance =
    gtsam::Matrix33::Identity() * params_.imu.integration_covariance;

  return imu_params;
}

void FactorGraphNode::incrementAverages()
{
  // Average IMU
  auto imu_msgs = imu_queue_.drain();
  for (const auto & msg : imu_msgs) {
    if (initial_imu_count_ == 0) {
      initial_imu_ = msg;
    } else {
      double n = static_cast<double>(initial_imu_count_ + 1);
      initial_imu_->linear_acceleration.x +=
        (msg->linear_acceleration.x - initial_imu_->linear_acceleration.x) / n;
      initial_imu_->linear_acceleration.y +=
        (msg->linear_acceleration.y - initial_imu_->linear_acceleration.y) / n;
      initial_imu_->linear_acceleration.z +=
        (msg->linear_acceleration.z - initial_imu_->linear_acceleration.z) / n;
      initial_imu_->angular_velocity.x +=
        (msg->angular_velocity.x - initial_imu_->angular_velocity.x) / n;
      initial_imu_->angular_velocity.y +=
        (msg->angular_velocity.y - initial_imu_->angular_velocity.y) / n;
      initial_imu_->angular_velocity.z +=
        (msg->angular_velocity.z - initial_imu_->angular_velocity.z) / n;
      initial_imu_->header.stamp = msg->header.stamp;
    }
    initial_imu_count_++;
  }

  // Average GPS
  if (params_.gps.enable_gps || params_.gps.enable_gps_init_only) {
    auto gps_msgs = gps_queue_.drain();
    for (const auto & msg : gps_msgs) {
      if (initial_gps_count_ == 0) {
        initial_gps_ = msg;
      } else {
        double n = static_cast<double>(initial_gps_count_ + 1);
        initial_gps_->pose.pose.position.x +=
          (msg->pose.pose.position.x - initial_gps_->pose.pose.position.x) / n;
        initial_gps_->pose.pose.position.y +=
          (msg->pose.pose.position.y - initial_gps_->pose.pose.position.y) / n;
        initial_gps_->pose.pose.position.z +=
          (msg->pose.pose.position.z - initial_gps_->pose.pose.position.z) / n;
        initial_gps_->header.stamp = msg->header.stamp;
      }
      initial_gps_count_++;
    }
  }

  // Average Depth
  auto depth_msgs = depth_queue_.drain();
  for (const auto & msg : depth_msgs) {
    if (initial_depth_count_ == 0) {
      initial_depth_ = msg;
    } else {
      double n = static_cast<double>(initial_depth_count_ + 1);
      initial_depth_->pose.pose.position.z +=
        (msg->pose.pose.position.z - initial_depth_->pose.pose.position.z) / n;
      initial_depth_->header.stamp = msg->header.stamp;
    }
    initial_depth_count_++;
  }

  // Average DVL
  auto dvl_msgs = dvl_queue_.drain();
  for (const auto & msg : dvl_msgs) {
    if (initial_dvl_count_ == 0) {
      initial_dvl_ = msg;
    } else {
      double n = static_cast<double>(initial_dvl_count_ + 1);
      initial_dvl_->twist.twist.linear.x +=
        (msg->twist.twist.linear.x - initial_dvl_->twist.twist.linear.x) / n;
      initial_dvl_->twist.twist.linear.y +=
        (msg->twist.twist.linear.y - initial_dvl_->twist.twist.linear.y) / n;
      initial_dvl_->twist.twist.linear.z +=
        (msg->twist.twist.linear.z - initial_dvl_->twist.twist.linear.z) / n;
      initial_dvl_->header.stamp = msg->header.stamp;
    }
    initial_dvl_count_++;
  }

  // Average Magnetometer
  if (params_.mag.enable_mag || params_.mag.enable_mag_init_only) {
    auto mag_msgs = mag_queue_.drain();
    for (const auto & msg : mag_msgs) {
      if (initial_mag_count_ == 0) {
        initial_mag_ = msg;
      } else {
        double n = static_cast<double>(initial_mag_count_ + 1);
        initial_mag_->magnetic_field.x +=
          (msg->magnetic_field.x - initial_mag_->magnetic_field.x) / n;
        initial_mag_->magnetic_field.y +=
          (msg->magnetic_field.y - initial_mag_->magnetic_field.y) / n;
        initial_mag_->magnetic_field.z +=
          (msg->magnetic_field.z - initial_mag_->magnetic_field.z) / n;
        initial_mag_->header.stamp = msg->header.stamp;
      }
      initial_mag_count_++;
    }
  }

  // Average AHRS
  if (params_.ahrs.enable_ahrs || params_.ahrs.enable_ahrs_init_only) {
    auto ahrs_msgs = ahrs_queue_.drain();
    for (const auto & msg : ahrs_msgs) {
      if (initial_ahrs_count_ == 0) {
        initial_ahrs_ = msg;
        initial_ahrs_ref_ = toGtsam(msg->orientation);
        initial_ahrs_log_sum_ = gtsam::Vector3::Zero();
      } else {
        initial_ahrs_log_sum_ +=
          gtsam::Rot3::Logmap(initial_ahrs_ref_.between(toGtsam(msg->orientation)));

        gtsam::Vector3 log_avg = initial_ahrs_log_sum_ /
          static_cast<double>(initial_ahrs_count_ + 1);
        initial_ahrs_->orientation =
          toQuatMsg(initial_ahrs_ref_.compose(gtsam::Rot3::Expmap(log_avg)));
        initial_ahrs_->header.stamp = msg->header.stamp;
      }
      initial_ahrs_count_++;
    }
  }

  // Set initial wrench
  if (params_.dynamics.enable_dynamics) {
    auto wrench_msgs = wrench_queue_.drain();
    if (!wrench_msgs.empty()) {
      initial_wrench_ = wrench_msgs.back();
      latest_wrench_msg_ = initial_wrench_;
    }
  }
}

gtsam::Rot3 FactorGraphNode::computeInitialOrientation()
{
  double roll = params_.prior.parameter_priors.initial_orientation[0];
  double pitch = params_.prior.parameter_priors.initial_orientation[1];
  double yaw = params_.prior.parameter_priors.initial_orientation[2];
  gtsam::Rot3 R_base_dvl = toGtsam(dvl_to_base_tf_.transform.rotation);

  if (params_.prior.use_parameter_priors) {
    // Account for DVL rotation
    gtsam::Rot3 R_world_base = gtsam::Rot3::Ypr(yaw, pitch, roll);
    return R_world_base * R_base_dvl;
  }

  // Account for IMU rotation
  gtsam::Vector3 accel_imu = toGtsam(initial_imu_->linear_acceleration);
  gtsam::Pose3 T_dvl_imu = toGtsam(imu_to_dvl_tf_.transform);
  gtsam::Pose3 T_base_dvl = toGtsam(dvl_to_base_tf_.transform);
  gtsam::Vector3 accel_base = T_base_dvl.rotation() * (T_dvl_imu.rotation() * accel_imu);

  roll = std::atan2(accel_base.y(), accel_base.z());
  pitch = std::atan2(
    -accel_base.x(), std::sqrt(
      accel_base.y() * accel_base.y() +
      accel_base.z() * accel_base.z()));

  if (params_.ahrs.enable_ahrs || params_.ahrs.enable_ahrs_init_only) {
    // Account for AHRS sensor rotation
    gtsam::Rot3 R_dvl_sensor = toGtsam(ahrs_to_dvl_tf_.transform.rotation);
    gtsam::Rot3 R_base_sensor = R_base_dvl * R_dvl_sensor;
    gtsam::Rot3 R_world_sensor = toGtsam(initial_ahrs_->orientation);
    gtsam::Rot3 R_world_base_measured = R_world_sensor * R_base_sensor.inverse();
    yaw = R_world_base_measured.yaw() + params_.ahrs.mag_declination_radians;
  } else if (params_.mag.enable_mag || params_.mag.enable_mag_init_only) {
    // Account for magnetometer rotation
    gtsam::Rot3 R_dvl_sensor = toGtsam(mag_to_dvl_tf_.transform.rotation);

    gtsam::Rot3 R_base_sensor = R_base_dvl * R_dvl_sensor;
    gtsam::Vector3 mag_sensor = toGtsam(initial_mag_->magnetic_field);
    gtsam::Vector3 mag_base = R_base_sensor * mag_sensor;

    // Use the tilt-compensated magnetic vector to calculate yaw
    gtsam::Rot3 R_rp = gtsam::Rot3::Ypr(0.0, pitch, roll);
    gtsam::Vector3 mag_horizontal = R_rp.unrotate(mag_base);

    double measured_yaw = std::atan2(mag_horizontal.y(), mag_horizontal.x());
    double ref_yaw = std::atan2(
      params_.mag.reference_field[1],
      params_.mag.reference_field[0]);

    yaw = ref_yaw - measured_yaw;
  }

  RCLCPP_DEBUG(get_logger(), "Initial orientation: %f %f %f", roll, pitch, yaw);

  return gtsam::Rot3::Ypr(yaw, pitch, roll) * R_base_dvl;
}

gtsam::Point3 FactorGraphNode::computeInitialPosition(const gtsam::Rot3 & initial_orientation_dvl)
{
  gtsam::Point3 P_world_base(
    params_.prior.parameter_priors.initial_position[0],
    params_.prior.parameter_priors.initial_position[1],
    params_.prior.parameter_priors.initial_position[2]);
  gtsam::Pose3 T_base_dvl = toGtsam(dvl_to_base_tf_.transform);
  gtsam::Rot3 R_world_base = initial_orientation_dvl * T_base_dvl.rotation().inverse();
  gtsam::Point3 P_world_dvl_param = P_world_base + R_world_base.rotate(T_base_dvl.translation());

  if (params_.prior.use_parameter_priors) {
    // Account for DVL lever arm
    return P_world_dvl_param;
  }

  gtsam::Point3 initial_position_dvl = P_world_dvl_param;
  if (params_.gps.enable_gps || params_.gps.enable_gps_init_only) {
    // Account for GPS lever arm
    gtsam::Pose3 T_dvl_gps = toGtsam(gps_to_dvl_tf_.transform);
    gtsam::Point3 world_t_dvl_gps = initial_orientation_dvl.rotate(T_dvl_gps.translation());
    initial_position_dvl = toGtsam(initial_gps_->pose.pose.position) - world_t_dvl_gps;
  }
  // Account for depth lever arm
  gtsam::Pose3 T_dvl_depth = toGtsam(depth_to_dvl_tf_.transform);
  gtsam::Point3 world_t_dvl_depth = initial_orientation_dvl.rotate(T_dvl_depth.translation());
  initial_position_dvl.z() = initial_depth_->pose.pose.position.z - world_t_dvl_depth.z();

  RCLCPP_DEBUG(
    get_logger(), "Initial position: %f %f %f", initial_position_dvl.x(),
    initial_position_dvl.y(), initial_position_dvl.z());

  return initial_position_dvl;
}

gtsam::Vector3 FactorGraphNode::computeInitialVelocity(const gtsam::Rot3 & initial_orientation_dvl)
{
  if (params_.prior.use_parameter_priors) {
    // Account for DVL rotation (in world frame)
    return initial_orientation_dvl.rotate(
      toGtsam(params_.prior.parameter_priors.initial_velocity));
  }

  RCLCPP_DEBUG(
    get_logger(), "Initial velocity: %f %f %f", initial_dvl_->twist.twist.linear.x,
    initial_dvl_->twist.twist.linear.y, initial_dvl_->twist.twist.linear.z);

  // Account for DVL rotation (in world frame)
  return initial_orientation_dvl.rotate(toGtsam(initial_dvl_->twist.twist.linear));
}

gtsam::imuBias::ConstantBias FactorGraphNode::computeInitialBias()
{
  gtsam::Vector3 init_gyro_bias;
  if (params_.prior.use_parameter_priors) {
    init_gyro_bias = toGtsam(params_.prior.parameter_priors.initial_gyro_bias);
  } else {
    init_gyro_bias = toGtsam(initial_imu_->angular_velocity);
  }
  gtsam::Vector3 init_accel_bias = toGtsam(params_.prior.parameter_priors.initial_accel_bias);

  RCLCPP_DEBUG(
    get_logger(), "Initial accel bias: %f %f %f", init_accel_bias.x(), init_accel_bias.y(),
    init_accel_bias.z());

  return gtsam::imuBias::ConstantBias(init_accel_bias, init_gyro_bias);
}

void FactorGraphNode::addPriorFactors(gtsam::NonlinearFactorGraph & graph, gtsam::Values & values)
{
  // Add initial pose prior
  gtsam::Vector6 prior_pose_sigmas;
  prior_pose_sigmas << params_.prior.parameter_priors.initial_orientation_sigmas[0],
    params_.prior.parameter_priors.initial_orientation_sigmas[1],
    params_.prior.parameter_priors.initial_orientation_sigmas[2],
    params_.prior.parameter_priors.initial_position_sigmas[0],
    params_.prior.parameter_priors.initial_position_sigmas[1],
    params_.prior.parameter_priors.initial_position_sigmas[2];

  if (!params_.prior.use_parameter_priors) {
    if (params_.gps.enable_gps) {
      prior_pose_sigmas(3) = params_.gps.use_parameter_covariance ?
        params_.gps.parameter_covariance.position_noise_sigmas[0] :
        std::sqrt(toGtsam(initial_gps_->pose.covariance)(0, 0));
      prior_pose_sigmas(4) = params_.gps.use_parameter_covariance ?
        params_.gps.parameter_covariance.position_noise_sigmas[1] :
        std::sqrt(toGtsam(initial_gps_->pose.covariance)(1, 1));
    }
    prior_pose_sigmas(5) = params_.depth.use_parameter_covariance ?
      params_.depth.parameter_covariance.position_z_noise_sigma :
      std::sqrt(initial_depth_->pose.covariance[14]);

    if (params_.ahrs.enable_ahrs) {
      prior_pose_sigmas(2) = params_.ahrs.use_parameter_covariance ?
        params_.ahrs.parameter_covariance.yaw_noise_sigma :
        std::sqrt(toGtsam(initial_ahrs_->orientation_covariance)(2, 2));
    } else if (params_.mag.enable_mag) {
      double h_mag = std::sqrt(
        params_.mag.reference_field[0] * params_.mag.reference_field[0] +
        params_.mag.reference_field[1] * params_.mag.reference_field[1]);

      double mag_sigma_norm = params_.mag.use_parameter_covariance ?
        params_.mag.parameter_covariance.magnetic_field_noise_sigmas[0] :
        std::sqrt(toGtsam(initial_mag_->magnetic_field_covariance)(0, 0));

      prior_pose_sigmas(2) = mag_sigma_norm / h_mag;
    }
    prior_pose_sigmas(0) = params_.imu.use_parameter_covariance ?
      params_.imu.parameter_covariance.gyro_noise_sigmas[0] :
      std::sqrt(toGtsam(initial_imu_->angular_velocity_covariance)(0, 0));
    prior_pose_sigmas(1) = params_.imu.use_parameter_covariance ?
      params_.imu.parameter_covariance.gyro_noise_sigmas[1] :
      std::sqrt(toGtsam(initial_imu_->angular_velocity_covariance)(1, 1));
  }

  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    X(0), prev_pose_, gtsam::noiseModel::Diagonal::Sigmas(prior_pose_sigmas));
  values.insert(X(0), prev_pose_);

  // Add initial velocity prior
  gtsam::SharedNoiseModel prior_vel_noise;
  if (params_.prior.use_parameter_priors) {
    auto & sigmas = params_.prior.parameter_priors.initial_velocity_sigmas;
    prior_vel_noise = gtsam::noiseModel::Diagonal::Sigmas(toGtsam(sigmas));
  } else {
    if (params_.dvl.use_parameter_covariance) {
      auto & sigmas = params_.dvl.parameter_covariance.velocity_noise_sigmas;
      prior_vel_noise = gtsam::noiseModel::Diagonal::Sigmas(toGtsam(sigmas));
    } else {
      gtsam::Matrix33 dvl_cov = toGtsam3x3(initial_dvl_->twist.covariance);
      prior_vel_noise = gtsam::noiseModel::Diagonal::Covariance(dvl_cov);
    }
  }

  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), prev_vel_, prior_vel_noise);
  values.insert(V(0), prev_vel_);

  // Add initial IMU bias prior
  gtsam::Vector6 prior_imu_bias_sigmas;
  gtsam::Vector3 accel_sigmas = toGtsam(params_.prior.initial_accel_bias_sigmas);
  gtsam::Vector3 gyro_sigmas = toGtsam(params_.prior.initial_gyro_bias_sigmas);
  prior_imu_bias_sigmas << accel_sigmas, gyro_sigmas;

  gtsam::SharedNoiseModel prior_imu_bias_noise =
    gtsam::noiseModel::Diagonal::Sigmas(prior_imu_bias_sigmas);

  graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
    B(0), prev_imu_bias_,
    prior_imu_bias_noise);
  values.insert(B(0), prev_imu_bias_);
}

void FactorGraphNode::initializeGraph()
{
  std::lock_guard<std::mutex> init_lock(initialization_mutex_);
  if (state_ == State::RUNNING) {
    RCLCPP_DEBUG(get_logger(), "Duplicate initialization attempt detected.");
    return;
  }

  bool perform_initialization = false;

  // --- Wait for Sensor Data ---
  if (state_ == State::WAITING_FOR_SENSORS) {
    bool imu_ok = !imu_queue_.empty() && have_imu_to_dvl_tf_;
    bool gps_ok = !(params_.gps.enable_gps || params_.gps.enable_gps_init_only) ||
      (!gps_queue_.empty() && have_gps_to_dvl_tf_);
    bool depth_ok = !depth_queue_.empty() && have_depth_to_dvl_tf_;
    bool mag_ok = !(params_.mag.enable_mag || params_.mag.enable_mag_init_only) ||
      (!mag_queue_.empty() && have_mag_to_dvl_tf_);
    bool ahrs_ok = !(params_.ahrs.enable_ahrs || params_.ahrs.enable_ahrs_init_only) ||
      (!ahrs_queue_.empty() && have_ahrs_to_dvl_tf_);
    bool dvl_ok = !dvl_queue_.empty() && have_dvl_to_base_tf_;
    bool wrench_ok = !params_.dynamics.enable_dynamics ||
      (!wrench_queue_.empty() && have_com_to_dvl_tf_);

    if (imu_ok && gps_ok && depth_ok && mag_ok && ahrs_ok && dvl_ok && wrench_ok) {
      if (params_.prior.use_parameter_priors) {
        RCLCPP_INFO(
          get_logger(),
          "Required sensor messages received! Skipping averaging...");
        initial_imu_ = imu_queue_.back();
        if (params_.gps.enable_gps || params_.gps.enable_gps_init_only) {
          initial_gps_ = gps_queue_.back();
        }
        initial_depth_ = depth_queue_.back();
        if (params_.mag.enable_mag || params_.mag.enable_mag_init_only) {
          initial_mag_ = mag_queue_.back();
        }
        if (params_.ahrs.enable_ahrs || params_.ahrs.enable_ahrs_init_only) {
          initial_ahrs_ = ahrs_queue_.back();
        }
        initial_dvl_ = dvl_queue_.back();
        if (params_.dynamics.enable_dynamics) {
          initial_wrench_ = wrench_queue_.back();
          latest_wrench_msg_ = initial_wrench_;
        }

        perform_initialization = true;
      } else {
        RCLCPP_INFO(get_logger(), "Required sensor messages received! Averaging...");
        start_avg_time_ = this->get_clock()->now().seconds();
        state_ = State::INITIALIZING;
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for sensors: %s%s%s%s%s%s%s",
        !imu_ok ? "[IMU] " : "",
        (!gps_ok && params_.gps.enable_gps) ? "[GPS] " : "",
        (!mag_ok && params_.mag.enable_mag) ? "[Magnetometer] " : "",
        (!ahrs_ok && params_.ahrs.enable_ahrs) ? "[AHRS] " : "",
        !depth_ok ? "[Depth] " : "", !dvl_ok ? "[DVL] " : "",
        (!wrench_ok && params_.dynamics.enable_dynamics) ? "[Wrench]" : "");
      return;
    }
  }

  // --- Average Initial Measurements ---
  if (state_ == State::INITIALIZING) {
    incrementAverages();

    double duration = this->get_clock()->now().seconds() - start_avg_time_;
    if (duration < params_.prior.initialization_duration) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Averaging sensor data (%.2fs / %.2fs)...", duration,
        params_.prior.initialization_duration);
      return;
    } else {
      RCLCPP_INFO(get_logger(), "Sensor data averaged successfully! Initializing graph...");
      perform_initialization = true;
    }
  }

  // --- Create Initial Graph ---
  if (perform_initialization) {
    gtsam::NonlinearFactorGraph initial_graph;
    gtsam::Values initial_values;

    gtsam::Rot3 initial_orientation_dvl = computeInitialOrientation();
    gtsam::Point3 initial_position_dvl = computeInitialPosition(initial_orientation_dvl);
    prev_pose_ = gtsam::Pose3(initial_orientation_dvl, initial_position_dvl);
    prev_vel_ = computeInitialVelocity(initial_orientation_dvl);
    prev_imu_bias_ = computeInitialBias();

    addPriorFactors(initial_graph, initial_values);

    rclcpp::Time init_stamp = params_.experimental.enable_dvl_preintegration ?
      initial_depth_->header.stamp :
      initial_dvl_->header.stamp;
    prev_time_ = init_stamp.seconds();
    time_to_key_[init_stamp] = X(0);

    // --- Initialize Preintegrators ---
    imu_preintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
      configureImuPreintegration(), prev_imu_bias_);
    if (params_.experimental.enable_dvl_preintegration) {
      dvl_preintegrator_ = std::make_unique<DvlPreintegrator>();
      dvl_preintegrator_->reset(initial_orientation_dvl);

      last_dvl_velocity_ = toGtsam(initial_dvl_->twist.twist.linear);
      if (params_.dvl.use_parameter_covariance) {
        last_dvl_covariance_ = toGtsamSquaredDiagonal(
          params_.dvl.parameter_covariance.velocity_noise_sigmas).block<3, 3>(0, 0);
      } else {
        last_dvl_covariance_ = toGtsam(initial_dvl_->twist.covariance).block<3, 3>(0, 0);
      }
    }

    // --- Initialize Smoother ---
    gtsam::IncrementalFixedLagSmoother::KeyTimestampMap initial_timestamps;
    initial_timestamps[X(0)] = prev_time_;
    initial_timestamps[V(0)] = prev_time_;
    initial_timestamps[B(0)] = prev_time_;

    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = params_.relinearize_threshold;
    isam2_params.relinearizeSkip = params_.relinearize_skip;
    if (params_.solver_type == "ISAM2") {
      isam_ = std::make_unique<gtsam::ISAM2>(isam2_params);
      isam_->update(initial_graph, initial_values);
    } else {
      inc_smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
        params_.smoother_lag,
        isam2_params);
      inc_smoother_->update(initial_graph, initial_values, initial_timestamps);
    }

    state_ = State::RUNNING;
    RCLCPP_INFO(get_logger(), "Graph initialized successfully!");
  }
}

void FactorGraphNode::addGpsFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<nav_msgs::msg::Odometry::SharedPtr> & gps_msgs)
{
  if (!have_gps_to_dvl_tf_ || gps_msgs.empty()) {return;}

  const auto & gps_msg = gps_msgs.back();

  gtsam::SharedNoiseModel gps_noise;
  if (params_.gps.use_parameter_covariance) {
    gps_noise = gtsam::noiseModel::Diagonal::Sigmas(
      toGtsam(params_.gps.parameter_covariance.position_noise_sigmas));
  } else {
    gtsam::Matrix22 gps_cov = toGtsam3x3(gps_msg->pose.covariance).block<2, 2>(0, 0);
    gps_noise = gtsam::noiseModel::Gaussian::Covariance(gps_cov);
  }

  if (params_.gps.robust_kernel == "Huber") {
    gps_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(params_.gps.robust_k), gps_noise);
  } else if (params_.gps.robust_kernel == "Tukey") {
    gps_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Tukey::Create(params_.gps.robust_k), gps_noise);
  }

  RCLCPP_DEBUG(get_logger(), "Adding GPS factor at step %zu", current_step_);

  graph.emplace_shared<Gps2dFactorArm>(
    X(current_step_), toGtsam(gps_msg->pose.pose.position),
    toGtsam(gps_to_dvl_tf_.transform), gps_noise);
}

void FactorGraphNode::addDepthFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<nav_msgs::msg::Odometry::SharedPtr> & depth_msgs)
{
  if (!have_depth_to_dvl_tf_ || depth_msgs.empty()) {return;}

  const auto & depth_msg = depth_msgs.back();

  gtsam::SharedNoiseModel depth_noise;
  if (params_.depth.use_parameter_covariance) {
    double depth_sigma = params_.depth.parameter_covariance.position_z_noise_sigma;
    depth_noise = gtsam::noiseModel::Isotropic::Sigma(1, depth_sigma);
  } else {
    gtsam::Matrix11 depth_cov;
    depth_cov << depth_msg->pose.covariance[14];
    depth_noise = gtsam::noiseModel::Gaussian::Covariance(depth_cov);
  }

  if (params_.depth.robust_kernel == "Huber") {
    depth_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(params_.depth.robust_k), depth_noise);
  } else if (params_.depth.robust_kernel == "Tukey") {
    depth_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Tukey::Create(params_.depth.robust_k), depth_noise);
  }

  RCLCPP_DEBUG(get_logger(), "Adding depth factor at step %zu", current_step_);

  graph.emplace_shared<DepthFactorArm>(
    X(current_step_), depth_msg->pose.pose.position.z,
    toGtsam(depth_to_dvl_tf_.transform), depth_noise);
}

void FactorGraphNode::addAhrsFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<sensor_msgs::msg::Imu::SharedPtr> & ahrs_msgs)
{
  if (!have_ahrs_to_dvl_tf_ || ahrs_msgs.empty()) {return;}

  const auto & ahrs_msg = ahrs_msgs.back();

  gtsam::SharedNoiseModel ahrs_noise;
  if (params_.ahrs.use_parameter_covariance) {
    gtsam::Vector1 ahrs_sigmas;
    ahrs_sigmas << params_.ahrs.parameter_covariance.yaw_noise_sigma;
    ahrs_noise = gtsam::noiseModel::Diagonal::Sigmas(ahrs_sigmas);
  } else {
    gtsam::Matrix11 ahrs_cov;
    ahrs_cov << ahrs_msg->orientation_covariance[8];
    ahrs_noise = gtsam::noiseModel::Gaussian::Covariance(ahrs_cov);
  }

  if (params_.ahrs.robust_kernel == "Huber") {
    ahrs_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(params_.ahrs.robust_k), ahrs_noise);
  } else if (params_.ahrs.robust_kernel == "Tukey") {
    ahrs_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Tukey::Create(params_.ahrs.robust_k), ahrs_noise);
  }

  RCLCPP_DEBUG(get_logger(), "Adding AHRS factor at step %zu", current_step_);

  graph.emplace_shared<AhrsYawFactorArm>(
    X(current_step_), toGtsam(ahrs_msg->orientation),
    toGtsam(ahrs_to_dvl_tf_.transform.rotation),
    params_.ahrs.mag_declination_radians,
    ahrs_noise);
}

void FactorGraphNode::addMagFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<sensor_msgs::msg::MagneticField::SharedPtr> & mag_msgs)
{
  if (!have_mag_to_dvl_tf_ || mag_msgs.empty()) {return;}

  const auto & mag_msg = mag_msgs.back();

  // IMPORTANT! The reference field must be in the world frame (ENU), not NED.
  // If getting values from NOAA (NED), convert as: [Y, X, -Z].
  gtsam::Point3 ref_vec(params_.mag.reference_field[0],
    params_.mag.reference_field[1],
    params_.mag.reference_field[2]);

  gtsam::SharedNoiseModel mag_noise;
  if (params_.mag.use_parameter_covariance) {
    mag_noise = gtsam::noiseModel::Diagonal::Sigmas(
      toGtsam(params_.mag.parameter_covariance.magnetic_field_noise_sigmas));
  } else {
    gtsam::Matrix33 mag_cov = toGtsam(mag_msg->magnetic_field_covariance);
    mag_noise = gtsam::noiseModel::Gaussian::Covariance(mag_cov);
  }

  if (params_.mag.robust_kernel == "Huber") {
    mag_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(params_.mag.robust_k), mag_noise);
  } else if (params_.mag.robust_kernel == "Tukey") {
    mag_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Tukey::Create(params_.mag.robust_k), mag_noise);
  }

  RCLCPP_DEBUG(get_logger(), "Adding mag factor at step %zu", current_step_);

  graph.emplace_shared<MagFactorArm>(
    X(current_step_), toGtsam(mag_msg->magnetic_field), ref_vec,
    toGtsam(mag_to_dvl_tf_.transform.rotation), mag_noise);
}

void FactorGraphNode::addDvlFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & dvl_msgs)
{
  if (dvl_msgs.empty()) {return;}

  const auto & dvl_msg = dvl_msgs.back();

  gtsam::SharedNoiseModel dvl_noise;
  if (params_.dvl.use_parameter_covariance) {
    dvl_noise = gtsam::noiseModel::Diagonal::Sigmas(
      toGtsam(params_.dvl.parameter_covariance.velocity_noise_sigmas));
  } else {
    gtsam::Matrix33 dvl_cov = toGtsam3x3(dvl_msg->twist.covariance);
    dvl_noise = gtsam::noiseModel::Gaussian::Covariance(dvl_cov);
  }

  if (params_.dvl.robust_kernel == "Huber") {
    dvl_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(params_.dvl.robust_k), dvl_noise);
  } else if (params_.dvl.robust_kernel == "Tukey") {
    dvl_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Tukey::Create(params_.dvl.robust_k), dvl_noise);
  }

  RCLCPP_DEBUG(get_logger(), "Adding DVL factor at step %zu", current_step_);

  graph.emplace_shared<DvlFactor>(
    X(current_step_), V(current_step_),
    toGtsam(dvl_msg->twist.twist.linear), dvl_noise);
}

void FactorGraphNode::addConstantVelocityFactor(
  gtsam::NonlinearFactorGraph & graph,
  double target_time)
{
  double dt = target_time - prev_time_;
  Eigen::Vector3d vel_random_walk = toGtsam(params_.const_vel.velocity_sigma);
  double sqrt_dt = std::sqrt(std::max(dt, 0.001));
  Eigen::Vector3d scaled_sigma = vel_random_walk * sqrt_dt;

  gtsam::SharedNoiseModel zero_accel_noise = gtsam::noiseModel::Diagonal::Sigmas(scaled_sigma);

  RCLCPP_DEBUG(get_logger(), "Adding constant velocity factor at step %zu", current_step_);

  graph.emplace_shared<ConstantVelocityFactor>(
    X(prev_step_), V(prev_step_),
    X(current_step_), V(current_step_),
    zero_accel_noise
  );
}

void FactorGraphNode::addAuvDynamicsFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<geometry_msgs::msg::WrenchStamped::SharedPtr> & wrench_msgs,
  double target_time)
{
  if (!have_com_to_dvl_tf_) {return;}

  // Implement a zero-order hold (ZOH) for wrench commands
  if (!wrench_msgs.empty()) {
    latest_wrench_msg_ = wrench_msgs.back();
  }

  if (!latest_wrench_msg_) {return;}

  const auto & wrench_msg = latest_wrench_msg_;

  gtsam::Vector3 dynamics_sigmas = toGtsam(params_.dynamics.prediction_noise_sigmas);
  gtsam::SharedNoiseModel dynamics_noise = gtsam::noiseModel::Diagonal::Sigmas(dynamics_sigmas);

  if (params_.dynamics.robust_kernel == "Huber") {
    dynamics_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(params_.dynamics.robust_k), dynamics_noise);
  } else if (params_.dynamics.robust_kernel == "Tukey") {
    dynamics_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Tukey::Create(params_.dynamics.robust_k), dynamics_noise);
  }

  double dt = target_time - prev_time_;
  RCLCPP_DEBUG(get_logger(), "Adding dynamics factor at step %zu", current_step_);
  graph.emplace_shared<coug_fgo::factors::AuvDynamicsFactorArm>(
    X(prev_step_), V(prev_step_),
    X(current_step_), V(current_step_),
    dt, toGtsam(wrench_msg->wrench.force),
    toGtsam(com_to_dvl_tf_.transform),
    toGtsamDiagonal(params_.dynamics.mass).block<3, 3>(0, 0),
    toGtsamDiagonal(params_.dynamics.linear_drag).block<3, 3>(0, 0),
    toGtsamDiagonal(params_.dynamics.quad_drag).block<3, 3>(0, 0),
    dynamics_noise);
}

void FactorGraphNode::addPreintegratedImuFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs, double target_time)
{
  if (!have_imu_to_dvl_tf_ || !imu_preintegrator_ || imu_msgs.empty()) {return;}

  double last_imu_time = prev_time_;
  std::deque<sensor_msgs::msg::Imu::SharedPtr> unused_imu_msgs;

  gtsam::Vector3 last_acc = gtsam::Vector3::Zero();
  gtsam::Vector3 last_gyr = gtsam::Vector3::Zero();

  for (const auto & imu_msg : imu_msgs) {
    double current_imu_time = rclcpp::Time(imu_msg->header.stamp).seconds();
    if (current_imu_time > target_time) {
      unused_imu_msgs.push_back(imu_msg);
      continue;
    }

    if (current_imu_time <= last_imu_time) {
      RCLCPP_DEBUG(get_logger(), "IMU message older than last integrated time. Skipping.");
      continue;
    }

    last_acc = toGtsam(imu_msg->linear_acceleration);
    last_gyr = toGtsam(imu_msg->angular_velocity);

    double dt = current_imu_time - last_imu_time;
    if (dt > 1e-9) {
      imu_preintegrator_->integrateMeasurement(last_acc, last_gyr, dt);
    }
    last_imu_time = current_imu_time;
  }

  if (last_imu_time < prev_time_) {
    RCLCPP_DEBUG(get_logger(), "No valid IMU measurements found. Skipping.");
    return;
  }

  // Extra measurement to reach exact target time
  if (last_imu_time < target_time) {
    double dt = target_time - last_imu_time;
    if (dt > 1e-6) {imu_preintegrator_->integrateMeasurement(last_acc, last_gyr, dt);}
    last_imu_time = target_time;
  }

  graph.emplace_shared<gtsam::CombinedImuFactor>(
    X(prev_step_), V(prev_step_), X(current_step_),
    V(current_step_), B(prev_step_),
    B(current_step_), *imu_preintegrator_);

  RCLCPP_DEBUG(get_logger(), "Adding preintegrated IMU factor at step %zu", current_step_);

  // Re-queue future IMU messages
  if (!unused_imu_msgs.empty()) {
    imu_queue_.restore(unused_imu_msgs);
  }
}

gtsam::Rot3 FactorGraphNode::getInterpolatedOrientation(
  const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs, double target_time)
{
  if (imu_msgs.empty()) {
    RCLCPP_DEBUG(get_logger(), "IMU queue empty. Returning identity rotation.");
    return gtsam::Rot3();
  }

  auto it_after = std::lower_bound(
    imu_msgs.begin(), imu_msgs.end(), target_time,
    [](const auto & msg, double t) {return rclcpp::Time(msg->header.stamp).seconds() < t;});

  if (it_after == imu_msgs.begin()) {return toGtsam(imu_msgs.front()->orientation);}

  // If past the last message, extrapolate into the future
  if (it_after == imu_msgs.end()) {
    if (imu_msgs.size() < 2) {return toGtsam(imu_msgs.back()->orientation);}
    it_after--;
  }

  double t1 = rclcpp::Time((*(it_after - 1))->header.stamp).seconds();
  double t2 = rclcpp::Time((*it_after)->header.stamp).seconds();
  double denominator = t2 - t1;

  if (std::abs(denominator) < 1e-9) {
    return toGtsam((*(it_after - 1))->orientation);
  }

  double alpha = (target_time - t1) / denominator;

  // Use Slerp for quaternion interpolation (handles alpha > 1.0 for extrapolation)
  return toGtsam((*(it_after - 1))->orientation).slerp(alpha, toGtsam((*it_after)->orientation));
}

void FactorGraphNode::addPreintegratedDvlFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & dvl_msgs,
  const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs, double target_time)
{
  if (!have_imu_to_dvl_tf_ || !dvl_preintegrator_ || imu_msgs.empty()) {return;}

  if (imu_msgs.empty()) {
    dvl_queue_.restore(dvl_msgs);
    return;
  }

  double last_dvl_time = prev_time_;
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> unused_dvl_msgs;

  gtsam::Rot3 R_dvl_to_imu = toGtsam(imu_to_dvl_tf_.transform.rotation);
  gtsam::Rot3 R_imu_to_dvl = R_dvl_to_imu.inverse();
  gtsam::Rot3 prev_imu_att = getInterpolatedOrientation(imu_msgs, prev_time_);
  dvl_preintegrator_->reset(prev_imu_att * R_imu_to_dvl);

  for (const auto & dvl_msg : dvl_msgs) {
    double current_dvl_time = rclcpp::Time(dvl_msg->header.stamp).seconds();
    if (current_dvl_time > target_time) {
      unused_dvl_msgs.push_back(dvl_msg);
      continue;
    }

    if (current_dvl_time <= last_dvl_time) {
      RCLCPP_DEBUG(get_logger(), "DVL message older than last integrated time. Skipping.");
      continue;
    }

    double dt = current_dvl_time - last_dvl_time;
    if (dt > 1e-9) {
      last_dvl_velocity_ = toGtsam(dvl_msg->twist.twist.linear);

      if (params_.dvl.use_parameter_covariance) {
        last_dvl_covariance_ = toGtsamSquaredDiagonal(
          params_.dvl.parameter_covariance.velocity_noise_sigmas).block<3, 3>(0, 0);
      } else {
        last_dvl_covariance_ = toGtsam3x3(dvl_msg->twist.covariance);
      }

      // Integrate DVL measurement alongside interpolated IMU attitude
      gtsam::Rot3 cur_imu_att = getInterpolatedOrientation(imu_msgs, current_dvl_time);
      gtsam::Rot3 cur_dvl_att = cur_imu_att * R_imu_to_dvl;

      dvl_preintegrator_->integrateMeasurement(
        last_dvl_velocity_, cur_dvl_att, dt,
        last_dvl_covariance_);
    }
    last_dvl_time = current_dvl_time;
  }

  if (last_dvl_time < target_time) {
    RCLCPP_DEBUG(get_logger(), "No valid DVL measurements found.");
  }

  // Extra measurement to reach exact target time
  if (last_dvl_time < target_time) {
    // TURTLMap-style DVL psuedo-measurement using IMU data
    if (params_.experimental.enable_pseudo_dvl_w_imu) {
      gtsam::PreintegratedCombinedMeasurements pim = *imu_preintegrator_;
      pim.resetIntegration();

      double current_loop_time = last_dvl_time;
      gtsam::Vector3 last_acc, last_gyr;

      for (const auto & msg : imu_msgs) {
        double imu_time = rclcpp::Time(msg->header.stamp).seconds();
        if (imu_time <= last_dvl_time) {continue;}
        if (imu_time > target_time) {break;}

        last_acc = toGtsam(msg->linear_acceleration);
        last_gyr = toGtsam(msg->angular_velocity);
        pim.integrateMeasurement(last_acc, last_gyr, imu_time - current_loop_time);
        current_loop_time = imu_time;
      }

      if (target_time > current_loop_time) {
        pim.integrateMeasurement(last_acc, last_gyr, target_time - current_loop_time);
      }

      gtsam::Rot3 start_imu_rot = getInterpolatedOrientation(imu_msgs, last_dvl_time);
      gtsam::NavState predicted_state = pim.predict(
        gtsam::NavState(
          gtsam::Pose3(start_imu_rot, gtsam::Point3()),
          (start_imu_rot * R_imu_to_dvl).rotate(last_dvl_velocity_)),
        prev_imu_bias_);

      gtsam::Vector3 dvl_body_vel = R_dvl_to_imu.unrotate(predicted_state.bodyVelocity());

      dvl_preintegrator_->integrateMeasurement(
        dvl_body_vel, predicted_state.attitude() * R_imu_to_dvl, target_time - last_dvl_time,
        gtsam::I_3x3 * 0.02);  // Hard-coded TURTLMap covariance

      last_dvl_velocity_ = dvl_body_vel;
    } else {
      double dt = target_time - last_dvl_time;
      if (dt > 1e-6) {
        gtsam::Rot3 cur_imu_att = getInterpolatedOrientation(imu_msgs, target_time);
        gtsam::Rot3 cur_dvl_att = cur_imu_att * R_imu_to_dvl;
        dvl_preintegrator_->integrateMeasurement(
          last_dvl_velocity_, cur_dvl_att, dt, last_dvl_covariance_);
      }
    }
    last_dvl_time = target_time;
  }

  RCLCPP_DEBUG(get_logger(), "Adding preintegrated DVL factor at step %zu", current_step_);

  graph.emplace_shared<DvlPreintegratedFactor>(
    X(prev_step_), X(current_step_), dvl_preintegrator_->delta(),
    gtsam::noiseModel::Gaussian::Covariance(dvl_preintegrator_->covariance()));

  // Re-queue future DVL messages
  if (!unused_dvl_msgs.empty()) {
    dvl_queue_.restore(unused_dvl_msgs);
  }
}

void FactorGraphNode::publishGlobalOdom(
  const gtsam::Pose3 & current_pose,
  const gtsam::Matrix & pose_covariance,
  const rclcpp::Time & timestamp)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = timestamp;
  odom_msg.header.frame_id = params_.map_frame;
  odom_msg.child_frame_id = params_.base_frame;
  odom_msg.pose.pose = toPoseMsg(current_pose);

  gtsam::Matrix cov_to_pub = pose_covariance;

  if (params_.publish_pose_cov) {
    gtsam::Rot3 R_map_dvl = current_pose.rotation();
    gtsam::Matrix66 Rot = gtsam::Matrix66::Zero();
    Rot.block<3, 3>(0, 0) = R_map_dvl.matrix();
    Rot.block<3, 3>(3, 3) = R_map_dvl.matrix();

    cov_to_pub = Rot * pose_covariance * Rot.transpose();
  }

  odom_msg.pose.covariance = toPoseCovarianceMsg(gtsam::Matrix66(cov_to_pub));
  odom_msg.twist.covariance[0] = -1.0;
  global_odom_pub_->publish(odom_msg);
}

void FactorGraphNode::broadcastGlobalTf(
  const gtsam::Pose3 & current_pose,
  const rclcpp::Time & timestamp)
{
  try {
    gtsam::Pose3 T_odom_base = toGtsam(
      tf_buffer_->lookupTransform(
        params_.odom_frame, params_.base_frame,
        tf2::TimePointZero).transform);
    gtsam::Pose3 T_map_odom = current_pose * T_odom_base.inverse();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = params_.map_frame;
    tf_msg.child_frame_id = params_.odom_frame;
    tf_msg.transform.translation = toVectorMsg(T_map_odom.translation());
    tf_msg.transform.rotation = toQuatMsg(T_map_odom.rotation());
    tf_broadcaster_->sendTransform(tf_msg);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Global TF lookup failed: %s",
      ex.what());
  }
}

void FactorGraphNode::publishSmoothedPath(
  const gtsam::Values & results,
  const rclcpp::Time & timestamp)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = timestamp;
  path_msg.header.frame_id = params_.map_frame;

  gtsam::Pose3 T_dvl_base = toGtsam(dvl_to_base_tf_.transform).inverse();

  for (const auto & pair : time_to_key_) {
    if (results.exists(pair.second)) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = params_.map_frame;
      ps.header.stamp = pair.first;
      ps.pose = toPoseMsg(results.at<gtsam::Pose3>(pair.second) * T_dvl_base);
      path_msg.poses.push_back(ps);
    }
  }
  smoothed_path_pub_->publish(path_msg);
}

void FactorGraphNode::publishVelocity(
  const gtsam::Vector3 & current_vel,
  const gtsam::Matrix & vel_covariance,
  const rclcpp::Time & timestamp)
{
  geometry_msgs::msg::TwistWithCovarianceStamped vel_msg;
  vel_msg.header.stamp = timestamp;

  // IMPORTANT! This is the velocity at the DVL with respect to the 'params_.map_frame'.
  vel_msg.header.frame_id = params_.map_frame;
  vel_msg.twist.twist.linear = toVectorMsg(current_vel);
  vel_msg.twist.covariance = toCovariance36Msg(gtsam::Matrix33(vel_covariance));

  for (int i = 3; i < 6; ++i) {
    vel_msg.twist.covariance[i * 6 + i] = -1.0;
  }
  velocity_pub_->publish(vel_msg);
}

void FactorGraphNode::publishImuBias(
  const gtsam::imuBias::ConstantBias & current_imu_bias,
  const gtsam::Matrix & imu_bias_covariance,
  const rclcpp::Time & timestamp)
{
  geometry_msgs::msg::TwistWithCovarianceStamped imu_bias_msg;
  imu_bias_msg.header.stamp = timestamp;
  imu_bias_msg.header.frame_id = imu_frame_;

  // IMPORTANT! We use 'linear' for accelerometer bias and 'angular' for gyroscope bias.
  imu_bias_msg.twist.twist.linear = toVectorMsg(current_imu_bias.accelerometer());
  imu_bias_msg.twist.twist.angular = toVectorMsg(current_imu_bias.gyroscope());
  imu_bias_msg.twist.covariance = toCovariance36Msg(gtsam::Matrix66(imu_bias_covariance));

  imu_bias_pub_->publish(imu_bias_msg);
}

void FactorGraphNode::publishGraphMetrics(const rclcpp::Time & timestamp)
{
  coug_fgo_msgs::msg::GraphMetrics metrics_msg;
  metrics_msg.header.stamp = timestamp;

  metrics_msg.opt_duration = last_opt_duration_.load();
  metrics_msg.prep_duration = last_prep_duration_.load();
  metrics_msg.update_duration = last_update_duration_.load();
  metrics_msg.cov_duration = last_cov_duration_.load();
  metrics_msg.new_factors = static_cast<uint32_t>(new_factors_.load());
  metrics_msg.total_factors = static_cast<uint32_t>(total_factors_.load());
  metrics_msg.total_variables = static_cast<uint32_t>(total_variables_.load());

  graph_metrics_pub_->publish(metrics_msg);
}

void FactorGraphNode::optimizeGraph()
{
  // --- Handle Processing Overflow ---
  std::unique_lock<std::mutex> opt_lock(optimization_mutex_, std::try_to_lock);
  if (!opt_lock.owns_lock()) {return;}

  auto opt_start = std::chrono::high_resolution_clock::now();
  rclcpp::Time target_stamp;
  bool should_abort = false;

  if (imu_queue_.empty()) {
    return;
  }

  if (params_.experimental.enable_dvl_preintegration) {
    if (depth_queue_.empty()) {return;}
    target_stamp = depth_queue_.back()->header.stamp;
  } else {
    if (dvl_queue_.empty() && depth_queue_.empty()) {return;}

    if (!dvl_queue_.empty()) {
      target_stamp = dvl_queue_.back()->header.stamp;
    } else {
      target_stamp = depth_queue_.back()->header.stamp;
    }
  }

  if (target_stamp.seconds() <= prev_time_ + 1e-6) {
    RCLCPP_DEBUG(get_logger(), "Duplicate or out-of-order timestamp detected. Skipping.");
    if (params_.experimental.enable_dvl_preintegration) {
      depth_queue_.pop_back();
    } else {
      if (!dvl_queue_.empty()) {dvl_queue_.pop_back();} else {depth_queue_.pop_back();}
    }
    should_abort = true;
  }

  if (!should_abort && params_.max_keyframe_rate > 0.0) {
    double min_period = 1.0 / params_.max_keyframe_rate;
    if (target_stamp.seconds() - prev_time_ < min_period) {
      RCLCPP_DEBUG(get_logger(), "Limiting keyframe rate. Skipping.");
      if (params_.experimental.enable_dvl_preintegration) {
        depth_queue_.pop_back();
      } else {
        if (!dvl_queue_.empty()) {dvl_queue_.pop_back();} else {depth_queue_.pop_back();}
      }
      should_abort = true;
    }
  }

  if (should_abort) {return;}

  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_msgs = imu_queue_.drain();
  std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_msgs = gps_queue_.drain();
  std::deque<nav_msgs::msg::Odometry::SharedPtr> depth_msgs = depth_queue_.drain();
  std::deque<sensor_msgs::msg::MagneticField::SharedPtr> mag_msgs = mag_queue_.drain();
  std::deque<sensor_msgs::msg::Imu::SharedPtr> ahrs_msgs = ahrs_queue_.drain();
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> dvl_msgs =
    dvl_queue_.drain();
  std::deque<geometry_msgs::msg::WrenchStamped::SharedPtr> wrench_msgs = wrench_queue_.drain();

  if (params_.experimental.enable_dvl_preintegration) {
    if (depth_msgs.size() > 1) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Processing overflow. Skipping %zu Depth keyframes.",
        depth_msgs.size() - 1);
      processing_overflow_ = true;
    } else {
      processing_overflow_ = false;
    }
  } else {
    if (dvl_msgs.size() > 1) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Processing overflow. Skipping %zu DVL keyframes.",
        dvl_msgs.size() - 1);
      processing_overflow_ = true;
    } else {
      processing_overflow_ = false;
    }
  }

  // Sort IMU (and DVL) messages
  auto by_time = [](const auto & a, const auto & b) {
      return rclcpp::Time(a->header.stamp) < rclcpp::Time(b->header.stamp);
    };
  std::sort(imu_msgs.begin(), imu_msgs.end(), by_time);
  if (params_.experimental.enable_dvl_preintegration) {
    std::sort(dvl_msgs.begin(), dvl_msgs.end(), by_time);
  }

  double target_time = target_stamp.seconds();

  // --- Create Factor Graph ---
  gtsam::NonlinearFactorGraph new_graph;
  gtsam::Values new_values;
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap new_timestamps;

  addPreintegratedImuFactor(new_graph, imu_msgs, target_time);
  if (params_.gps.enable_gps) {addGpsFactor(new_graph, gps_msgs);}
  addDepthFactor(new_graph, depth_msgs);
  if (params_.mag.enable_mag) {addMagFactor(new_graph, mag_msgs);}
  if (params_.ahrs.enable_ahrs) {addAhrsFactor(new_graph, ahrs_msgs);}

  bool dvl_available = !dvl_msgs.empty();

  if (params_.experimental.enable_dvl_preintegration) {
    if (!dvl_available && !params_.experimental.enable_pseudo_dvl_w_imu) {
      bool use_dynamics = params_.dynamics.enable_dynamics ||
        (params_.dynamics.enable_dynamics_dropout_only && !dvl_available);
      bool use_const_vel = params_.const_vel.enable_const_vel ||
        (params_.const_vel.enable_const_vel_dropout_only && !dvl_available);

      if (use_dynamics) {
        addAuvDynamicsFactor(new_graph, wrench_msgs, target_time);
      } else if (use_const_vel) {
        addConstantVelocityFactor(new_graph, target_time);
      }
    } else {
      addPreintegratedDvlFactor(new_graph, dvl_msgs, imu_msgs, target_time);
    }
  } else {
    if (!dvl_available) {
      bool use_dynamics = params_.dynamics.enable_dynamics ||
        (params_.dynamics.enable_dynamics_dropout_only && !dvl_available);
      bool use_const_vel = params_.const_vel.enable_const_vel ||
        (params_.const_vel.enable_const_vel_dropout_only && !dvl_available);

      if (use_dynamics) {
        addAuvDynamicsFactor(new_graph, wrench_msgs, target_time);
      } else if (use_const_vel) {
        addConstantVelocityFactor(new_graph, target_time);
      }
    } else {
      addDvlFactor(new_graph, dvl_msgs);

      if (params_.dynamics.enable_dynamics) {
        addAuvDynamicsFactor(new_graph, wrench_msgs, target_time);
      } else if (params_.const_vel.enable_const_vel) {
        addConstantVelocityFactor(new_graph, target_time);
      }
    }
  }

  // --- Smoother Update ---
  auto pred = imu_preintegrator_->predict(gtsam::NavState(prev_pose_, prev_vel_), prev_imu_bias_);
  new_values.insert(X(current_step_), pred.pose());
  new_values.insert(V(current_step_), pred.velocity());
  new_values.insert(B(current_step_), prev_imu_bias_);
  new_timestamps[X(current_step_)] = target_time;
  new_timestamps[V(current_step_)] = target_time;
  new_timestamps[B(current_step_)] = target_time;

  try {
    auto prep_end = std::chrono::high_resolution_clock::now();
    last_prep_duration_ = std::chrono::duration<double>(prep_end - opt_start).count();
    new_factors_ = new_graph.size();

    if (inc_smoother_) {
      auto update_start = std::chrono::high_resolution_clock::now();
      inc_smoother_->update(new_graph, new_values, new_timestamps);
      auto update_end = std::chrono::high_resolution_clock::now();
      last_update_duration_ = std::chrono::duration<double>(update_end - update_start).count();

      prev_pose_ = inc_smoother_->calculateEstimate<gtsam::Pose3>(X(current_step_));
      prev_vel_ = inc_smoother_->calculateEstimate<gtsam::Vector3>(V(current_step_));
      prev_imu_bias_ =
        inc_smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(current_step_));

      if (params_.publish_diagnostics || params_.publish_graph_metrics) {
        total_factors_ = inc_smoother_->getFactors().nrFactors();
        total_variables_ = inc_smoother_->getLinearizationPoint().size();
      }

    } else if (isam_) {
      auto update_start = std::chrono::high_resolution_clock::now();
      isam_->update(new_graph, new_values);
      auto update_end = std::chrono::high_resolution_clock::now();
      last_update_duration_ = std::chrono::duration<double>(update_end - update_start).count();

      prev_pose_ = isam_->calculateEstimate<gtsam::Pose3>(X(current_step_));
      prev_vel_ = isam_->calculateEstimate<gtsam::Vector3>(V(current_step_));
      prev_imu_bias_ =
        isam_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(current_step_));

      if (params_.publish_diagnostics || params_.publish_graph_metrics) {
        total_factors_ = isam_->getFactorsUnsafe().nrFactors();
        total_variables_ = isam_->getLinearizationPoint().size();
      }
    }

    imu_preintegrator_->resetIntegrationAndSetBias(prev_imu_bias_);

    time_to_key_[target_stamp] = X(current_step_);
    if (!isam_) {
      time_to_key_.erase(
        time_to_key_.begin(),
        time_to_key_.lower_bound(
          target_stamp -
          rclcpp::Duration::from_seconds(params_.smoother_lag)));
    }

    // --- Calculate Covariances ---
    auto cov_start = std::chrono::high_resolution_clock::now();
    gtsam::Matrix new_pose_cov = gtsam::Matrix::Identity(6, 6) * -1.0;
    if (params_.publish_pose_cov) {
      if (inc_smoother_) {
        new_pose_cov = inc_smoother_->marginalCovariance(X(current_step_));
      } else if (isam_) {
        new_pose_cov = isam_->marginalCovariance(X(current_step_));
      }
    }

    gtsam::Matrix vel_cov = gtsam::Matrix::Identity(3, 3) * -1.0;
    if (params_.publish_velocity && params_.publish_velocity_cov) {
      if (inc_smoother_) {
        vel_cov = inc_smoother_->marginalCovariance(V(current_step_));
      } else if (isam_) {
        vel_cov = isam_->marginalCovariance(V(current_step_));
      }
    }

    gtsam::Matrix bias_cov = gtsam::Matrix::Identity(6, 6) * -1.0;
    if (params_.publish_imu_bias && params_.publish_imu_bias_cov) {
      if (inc_smoother_) {
        bias_cov = inc_smoother_->marginalCovariance(B(current_step_));
      } else if (isam_) {
        bias_cov = isam_->marginalCovariance(B(current_step_));
      }
    }
    auto opt_end = std::chrono::high_resolution_clock::now();
    last_cov_duration_ = std::chrono::duration<double>(opt_end - cov_start).count();
    last_opt_duration_ = std::chrono::duration<double>(opt_end - opt_start).count();

    // --- Publish Global Odometry ---
    gtsam::Pose3 T_base_dvl = toGtsam(dvl_to_base_tf_.transform);
    publishGlobalOdom(prev_pose_ * T_base_dvl.inverse(), new_pose_cov, target_stamp);

    // --- Publish Global TF ---
    if (params_.publish_global_tf) {
      broadcastGlobalTf(prev_pose_ * T_base_dvl.inverse(), target_stamp);
    }

    // --- Publish Smoothed Path ---
    if (params_.publish_smoothed_path) {
      if (inc_smoother_) {
        publishSmoothedPath(inc_smoother_->calculateEstimate(), target_stamp);
      } else if (isam_) {
        publishSmoothedPath(isam_->calculateEstimate(), target_stamp);
      }
    }

    // --- Publish Velocity ---
    if (params_.publish_velocity) {
      publishVelocity(prev_vel_, vel_cov, target_stamp);
    }

    // --- Publish IMU Bias ---
    if (params_.publish_imu_bias) {
      publishImuBias(prev_imu_bias_, bias_cov, target_stamp);
    }

    // --- Publish Graph Metrics ---
    if (params_.publish_graph_metrics) {
      publishGraphMetrics(target_stamp);
    }

    prev_time_ = target_time;
    prev_step_ = current_step_;
    current_step_++;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "%s", e.what());
    rclcpp::shutdown();
  }
}

void FactorGraphNode::checkSensorInputs(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All requested sensors online.");

  auto check_queue =
    [&](const std::string & name, size_t size, double last_time, bool enabled, bool is_critical,
      double timeout) {
      if (!enabled) {return;}

      double time_since =
        (last_time > 0.0) ? (this->get_clock()->now().seconds() - last_time) : -1.0;

      stat.add(name + " Queue Size", size);
      stat.add(name + " Time Since Last (s)", time_since);

      if (time_since > timeout || (last_time == 0.0 && size == 0)) {
        if (is_critical) {
          stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, name + " is offline.");
        } else {
          stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, name + " is offline.");
        }
      }
    };

  check_queue(
    "IMU", imu_queue_.size(), imu_queue_.getLastTime(), true, true,
    params_.imu.timeout_threshold);
  check_queue(
    "GPS", gps_queue_.size(), gps_queue_.getLastTime(), params_.gps.enable_gps, false,
    params_.gps.timeout_threshold);
  check_queue(
    "Depth", depth_queue_.size(), depth_queue_.getLastTime(), true, true,
    params_.depth.timeout_threshold);
  check_queue(
    "Mag", mag_queue_.size(), mag_queue_.getLastTime(), params_.mag.enable_mag, false,
    params_.mag.timeout_threshold);
  check_queue(
    "AHRS", ahrs_queue_.size(), ahrs_queue_.getLastTime(), params_.ahrs.enable_ahrs, false,
    params_.ahrs.timeout_threshold);
  check_queue(
    "DVL", dvl_queue_.size(), dvl_queue_.getLastTime(), true, true,
    params_.dvl.timeout_threshold);
  check_queue(
    "Wrench", wrench_queue_.size(), wrench_queue_.getLastTime(), params_.dynamics.enable_dynamics,
    false, params_.dynamics.timeout_threshold);
}

void FactorGraphNode::checkGraphState(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  switch (state_) {
    case State::RUNNING:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Optimizing factor graph.");
      break;
    case State::INITIALIZING:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Initializing factor graph.");
      break;
    case State::WAITING_FOR_SENSORS:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for sensor data.");
      break;
  }
}

void FactorGraphNode::checkProcessingOverflow(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (processing_overflow_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Processing overflow detected. Skipping keyframes.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No processing overflow detected.");
  }
  stat.add("Optimization Duration (s)", last_opt_duration_.load());
  stat.add("Data Prep Duration (s)", last_prep_duration_.load());
  stat.add("Update Duration (s)", last_update_duration_.load());
  stat.add("Covariance Duration (s)", last_cov_duration_.load());

  stat.add("New Factors", new_factors_.load());
  stat.add("Total Factors", total_factors_.load());
  stat.add("Total Variables", total_variables_.load());
}

}  // namespace coug_fgo

RCLCPP_COMPONENTS_REGISTER_NODE(coug_fgo::FactorGraphNode)
