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

#include "coug_fgo/factors/depth_factor_arm.hpp"
#include "coug_fgo/factors/dvl_factor.hpp"
#include "coug_fgo/factors/dvl_preintegrated_factor.hpp"
#include "coug_fgo/factors/gps_factor_arm.hpp"
#include "coug_fgo/factors/ahrs_factor.hpp"
#include "coug_fgo/factors/mag_factor_arm.hpp"
#include "coug_fgo/utils/conversion_utils.hpp"

using coug_fgo::factors::CustomDepthFactorArm;
using coug_fgo::factors::CustomDVLFactor;
using coug_fgo::factors::CustomDVLPreintegratedFactor;
using coug_fgo::factors::CustomGPSFactorArm;
using coug_fgo::factors::AhrsFactor;
using coug_fgo::factors::CustomMagFactorArm;
using coug_fgo::utils::toGtsam;
using coug_fgo::utils::toQuatMsg;
using coug_fgo::utils::toPointMsg;
using coug_fgo::utils::toVectorMsg;
using coug_fgo::utils::toPoseMsg;
using coug_fgo::utils::DVLPreintegrator;

using gtsam::symbol_shorthand::B;  // Bias (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo
{

void FactorGraphNode::loadParameters()
{
  // --- Node Settings ---
  publish_global_tf_ = declare_parameter<bool>("publish_global_tf", true);
  publish_smoothed_path_ = declare_parameter<bool>("publish_smoothed_path", false);
  publish_velocity_ = declare_parameter<bool>("publish_velocity", false);
  publish_imu_bias_ = declare_parameter<bool>("publish_imu_bias", false);

  publish_pose_cov_ = declare_parameter<bool>("publish_pose_cov", false);
  publish_velocity_cov_ = declare_parameter<bool>("publish_velocity_cov", false);
  publish_imu_bias_cov_ = declare_parameter<bool>("publish_imu_bias_cov", false);

  // --- GTSAM Settings ---
  smoother_lag_ = declare_parameter<double>("smoother_lag", 10.0);
  gtsam_relinearize_threshold_ = declare_parameter<double>("relinearize_threshold", 0.1);
  gtsam_relinearize_skip_ = declare_parameter<int>("relinearize_skip", 1);

  // --- ROS Topics and Frames ---
  imu_topic_ = declare_parameter<std::string>("imu_topic", "imu/data");
  gps_odom_topic_ = declare_parameter<std::string>("gps_odom_topic", "odometry/gps");
  depth_odom_topic_ = declare_parameter<std::string>("depth_odom_topic", "odometry/depth");
  mag_topic_ = declare_parameter<std::string>("mag_topic", "imu/mag");
  ahrs_topic_ = declare_parameter<std::string>("ahrs_topic", "imu/ahrs");
  dvl_topic_ = declare_parameter<std::string>("dvl_topic", "dvl/data");
  global_odom_topic_ = declare_parameter<std::string>("global_odom_topic", "odometry/global");
  smoothed_path_topic_ = declare_parameter<std::string>("smoothed_path_topic", "smoothed_path");
  velocity_topic_ = declare_parameter<std::string>("velocity_topic", "~/velocity");
  imu_bias_topic_ = declare_parameter<std::string>("imu_bias_topic", "~/imu_bias");

  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

  // --- Sensor Settings ---
  imu_params_.use_parameter_covariance =
    declare_parameter<bool>("imu.use_parameter_covariance", false);
  imu_params_.accel_noise_sigma =
    declare_parameter<double>("imu.parameter_covariance.accel_noise_sigma", 0.0078);
  imu_params_.gyro_noise_sigma =
    declare_parameter<double>("imu.parameter_covariance.gyro_noise_sigma", 0.0012);
  imu_params_.accel_bias_rw_sigma =
    declare_parameter<double>("imu.accel_bias_rw_sigma", 1.05e-04);
  imu_params_.gyro_bias_rw_sigma = declare_parameter<double>("imu.gyro_bias_rw_sigma", 3.91e-05);
  imu_params_.integration_covariance =
    declare_parameter<double>("imu.integration_covariance", 1e-5);
  imu_params_.gravity = declare_parameter<std::vector<double>>("imu.gravity", {0.0, 0.0, -9.8});

  gps_params_.enable = declare_parameter<bool>("gps.enable_gps", false);
  gps_params_.use_parameter_covariance =
    declare_parameter<bool>("gps.use_parameter_covariance", false);
  gps_params_.position_noise_sigma =
    declare_parameter<double>("gps.parameter_covariance.position_noise_sigma", 0.2);
  gps_params_.altitude_noise_sigma =
    declare_parameter<double>("gps.parameter_covariance.altitude_noise_sigma", 0.4);
  gps_params_.robust_kernel = declare_parameter<std::string>("gps.robust_kernel", "None");
  gps_params_.robust_k = declare_parameter<double>("gps.robust_k", 1.345);

  depth_params_.use_parameter_covariance =
    declare_parameter<bool>("depth.use_parameter_covariance", false);
  depth_params_.position_z_noise_sigma =
    declare_parameter<double>("depth.parameter_covariance.position_z_noise_sigma", 0.02);
  depth_params_.robust_kernel = declare_parameter<std::string>("depth.robust_kernel", "None");
  depth_params_.robust_k = declare_parameter<double>("depth.robust_k", 1.345);

  mag_params_.enable = declare_parameter<bool>("mag.enable_mag", false);
  mag_params_.constrain_yaw_only = declare_parameter<bool>("mag.constrain_yaw_only", true);
  mag_params_.use_parameter_covariance =
    declare_parameter<bool>("mag.use_parameter_covariance", false);
  mag_params_.magnetic_field_noise_sigma =
    declare_parameter<double>("mag.parameter_covariance.magnetic_field_noise_sigma", 0.003);
  mag_params_.reference_field =
    declare_parameter<std::vector<double>>("mag.reference_field", {0.41, 0.08, 0.91});
  mag_params_.robust_kernel = declare_parameter<std::string>("mag.robust_kernel", "None");
  mag_params_.robust_k = declare_parameter<double>("mag.robust_k", 1.345);

  ahrs_params_.enable_ahrs = declare_parameter<bool>("ahrs.enable_ahrs", false);
  ahrs_params_.use_parameter_covariance =
    declare_parameter<bool>("ahrs.use_parameter_covariance", false);
  ahrs_params_.yaw_noise_sigma =
    declare_parameter<double>("ahrs.parameter_covariance.yaw_noise_sigma", 0.01745);
  ahrs_params_.roll_pitch_noise_sigma =
    declare_parameter<double>("ahrs.parameter_covariance.roll_pitch_noise_sigma", 0.00349);
  ahrs_params_.robust_kernel = declare_parameter<std::string>("ahrs.robust_kernel", "None");
  ahrs_params_.robust_k = declare_parameter<double>("ahrs.robust_k", 1.345);

  dvl_params_.use_parameter_covariance =
    declare_parameter<bool>("dvl.use_parameter_covariance", false);
  dvl_params_.velocity_noise_sigma =
    declare_parameter<double>("dvl.parameter_covariance.velocity_noise_sigma", 0.02);
  dvl_params_.timeout_threshold =
    declare_parameter<double>("dvl.timeout_threshold", 1.0);
  dvl_params_.robust_kernel = declare_parameter<std::string>("dvl.robust_kernel", "None");
  dvl_params_.robust_k = declare_parameter<double>("dvl.robust_k", 1.345);

  // --- Prior Settings ---
  prior_params_.initialization_duration =
    declare_parameter<double>("prior.initialization_duration", 10.0);

  prior_params_.use_parameter_priors =
    declare_parameter<bool>("prior.use_parameter_priors", false);
  prior_params_.initial_position = declare_parameter<std::vector<double>>(
    "prior.parameter_priors.initial_position", {0.0, 0.0, 0.0});
  prior_params_.initial_orientation = declare_parameter<std::vector<double>>(
    "prior.parameter_priors.initial_orientation", {0.0, 0.0, 0.0});
  prior_params_.initial_velocity = declare_parameter<std::vector<double>>(
    "prior.parameter_priors.initial_velocity", {0.0, 0.0, 0.0});
  prior_params_.initial_accel_bias = declare_parameter<std::vector<double>>(
    "prior.parameter_priors.initial_accel_bias", {0.0, 0.0, 0.0});
  prior_params_.initial_gyro_bias = declare_parameter<std::vector<double>>(
    "prior.parameter_priors.initial_gyro_bias", {0.0, 0.0, 0.0});
  prior_params_.initial_position_sigma =
    declare_parameter<double>("prior.parameter_priors.initial_position_sigma", 1e-3);
  prior_params_.initial_orientation_sigma =
    declare_parameter<double>("prior.parameter_priors.initial_orientation_sigma", 1e-3);
  prior_params_.initial_velocity_sigma =
    declare_parameter<double>("prior.parameter_priors.initial_velocity_sigma", 1e-3);
  prior_params_.initial_accel_bias_sigma =
    declare_parameter<double>("prior.initial_accel_bias_sigma", 1e-3);
  prior_params_.initial_gyro_bias_sigma =
    declare_parameter<double>("prior.initial_gyro_bias_sigma", 1e-4);

  // --- Experimental Features ---
  experimental_params_.enable_dvl_preintegration =
    declare_parameter<bool>("experimental.enable_dvl_preintegration", false);
}

void FactorGraphNode::setupRosInterfaces()
{
  // --- ROS TF Interfaces ---
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // --- ROS Publishers ---
  global_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(global_odom_topic_, 10);
  smoothed_path_pub_ = create_publisher<nav_msgs::msg::Path>(smoothed_path_topic_, 10);
  velocity_pub_ =
    create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(velocity_topic_, 10);
  imu_bias_pub_ =
    create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(imu_bias_topic_, 10);

  // --- ROS Callback Groups ---
  sensor_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sensor_options = rclcpp::SubscriptionOptions();
  sensor_options.callback_group = sensor_cb_group_;

  // --- ROS Subscribers ---
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 200,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      std::scoped_lock lock(imu_queue_mutex_);
      imu_queue_.push_back(msg);
    },
    sensor_options);

  gps_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    gps_odom_topic_, 20,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      std::scoped_lock lock(gps_queue_mutex_);
      gps_queue_.push_back(msg);
    },
    sensor_options);

  depth_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    depth_odom_topic_, 20,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      {
        std::scoped_lock lock(depth_queue_mutex_);
        depth_queue_.push_back(msg);
      }

      // IMPORTANT! Detect DVL dropouts and use depth as the driver
      double time_since_dvl = this->get_clock()->now().seconds() - last_dvl_time_;
      bool dvl_timed_out = time_since_dvl > dvl_params_.timeout_threshold;

      if (experimental_params_.enable_dvl_preintegration || dvl_timed_out) {
        if (!graph_initialized_) {
          initializeGraph();
        } else {
          optimizeGraph();
        }
      }
    },
    sensor_options);

  mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
    mag_topic_, 20,
    [this](const sensor_msgs::msg::MagneticField::SharedPtr msg) {
      std::scoped_lock lock(mag_queue_mutex_);
      mag_queue_.push_back(msg);
    },
    sensor_options);

  ahrs_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    ahrs_topic_, 20,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      std::scoped_lock lock(ahrs_queue_mutex_);
      ahrs_queue_.push_back(msg);
    },
    sensor_options);

  dvl_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    dvl_topic_, 20,
    [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
      {
        std::scoped_lock lock(dvl_queue_mutex_);
        dvl_queue_.push_back(msg);
      }
      last_dvl_time_ = this->get_clock()->now().seconds();

      if (!experimental_params_.enable_dvl_preintegration) {
        if (!graph_initialized_) {
          initializeGraph();
        } else {
          optimizeGraph();
        }
      }
    },
    sensor_options);
}

FactorGraphNode::FactorGraphNode()
: Node("factor_graph_node")
{
  RCLCPP_INFO(get_logger(), "Starting Factor Graph Node...");
  loadParameters();
  setupRosInterfaces();
  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for sensor messages...");
}

bool FactorGraphNode::lookupInitialTransforms()
{
  try {
    dvl_frame_ = initial_dvl_->header.frame_id;
    imu_frame_ = initial_imu_->header.frame_id;

    auto lookup = [&](const std::string & parent, const std::string & child) {
        return tf_buffer_->lookupTransform(
          parent, child, tf2::TimePointZero,
          tf2::durationFromSec(0.1));
      };

    if (!have_dvl_to_base_tf_) {
      dvl_to_base_tf_ = lookup(base_frame_, dvl_frame_);
      have_dvl_to_base_tf_ = true;
    }
    if (!have_imu_to_dvl_tf_) {
      imu_to_dvl_tf_ = lookup(dvl_frame_, initial_imu_->header.frame_id);
      have_imu_to_dvl_tf_ = true;
    }
    if (gps_params_.enable && !have_gps_to_dvl_tf_) {
      gps_to_dvl_tf_ = lookup(dvl_frame_, initial_gps_->child_frame_id);
      have_gps_to_dvl_tf_ = true;
    }
    if (!have_depth_to_dvl_tf_) {
      depth_to_dvl_tf_ = lookup(dvl_frame_, initial_depth_->child_frame_id);
      have_depth_to_dvl_tf_ = true;
    }
    if (mag_params_.enable && !have_mag_to_dvl_tf_) {
      mag_to_dvl_tf_ = lookup(dvl_frame_, initial_mag_->header.frame_id);
      have_mag_to_dvl_tf_ = true;
    }
    if (ahrs_params_.enable_ahrs && !have_ahrs_to_dvl_tf_) {
      ahrs_to_dvl_tf_ = lookup(dvl_frame_, initial_ahrs_->header.frame_id);
      have_ahrs_to_dvl_tf_ = true;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Could not get required transforms: %s", ex.what());
    return false;
  }
  return true;
}

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
FactorGraphNode::configureImuPreintegration()
{
  auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
  imu_params->n_gravity =
    gtsam::Vector3(imu_params_.gravity[0], imu_params_.gravity[1], imu_params_.gravity[2]);
  imu_params->body_P_sensor = toGtsam(imu_to_dvl_tf_.transform);

  auto set_cov = [&](bool use_param, double sigma, const std::array<double, 9> & msg_cov) {
      return use_param ? (gtsam::Matrix33)(gtsam::Matrix33::Identity() * (sigma * sigma)) :
             gtsam::Matrix33(msg_cov.data());
    };

  imu_params->accelerometerCovariance =
    set_cov(
    imu_params_.use_parameter_covariance, imu_params_.accel_noise_sigma,
    initial_imu_->linear_acceleration_covariance);
  imu_params->gyroscopeCovariance =
    set_cov(
    imu_params_.use_parameter_covariance, imu_params_.gyro_noise_sigma,
    initial_imu_->angular_velocity_covariance);
  imu_params->biasAccCovariance = gtsam::Matrix33::Identity() * (imu_params_.accel_bias_rw_sigma *
    imu_params_.accel_bias_rw_sigma);
  imu_params->biasOmegaCovariance =
    gtsam::Matrix33::Identity() *
    (imu_params_.gyro_bias_rw_sigma * imu_params_.gyro_bias_rw_sigma);
  imu_params->integrationCovariance =
    gtsam::Matrix33::Identity() * imu_params_.integration_covariance;

  return imu_params;
}

FactorGraphNode::AveragedMeasurements FactorGraphNode::computeAveragedMeasurements(
  const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs,
  const std::deque<nav_msgs::msg::Odometry::SharedPtr> & gps_msgs,
  const std::deque<nav_msgs::msg::Odometry::SharedPtr> & depth_msgs,
  const std::deque<sensor_msgs::msg::MagneticField::SharedPtr> & mag_msgs,
  const std::deque<sensor_msgs::msg::Imu::SharedPtr> & ahrs_msgs,
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & dvl_msgs)
{
  AveragedMeasurements result;

  // Average IMU
  result.imu = std::make_shared<sensor_msgs::msg::Imu>(*imu_msgs.back());
  result.imu->linear_acceleration.x = 0;
  result.imu->linear_acceleration.y = 0;
  result.imu->linear_acceleration.z = 0;
  result.imu->angular_velocity.x = 0;
  result.imu->angular_velocity.y = 0;
  result.imu->angular_velocity.z = 0;

  for (const auto & m : imu_msgs) {
    result.imu->linear_acceleration.x += m->linear_acceleration.x;
    result.imu->linear_acceleration.y += m->linear_acceleration.y;
    result.imu->linear_acceleration.z += m->linear_acceleration.z;
    result.imu->angular_velocity.x += m->angular_velocity.x;
    result.imu->angular_velocity.y += m->angular_velocity.y;
    result.imu->angular_velocity.z += m->angular_velocity.z;
  }
  double n_imu = imu_msgs.size();
  result.imu->linear_acceleration.x /= n_imu;
  result.imu->linear_acceleration.y /= n_imu;
  result.imu->linear_acceleration.z /= n_imu;
  result.imu->angular_velocity.x /= n_imu;
  result.imu->angular_velocity.y /= n_imu;
  result.imu->angular_velocity.z /= n_imu;

  // Average GPS
  if (gps_params_.enable) {
    result.gps = std::make_shared<nav_msgs::msg::Odometry>(*gps_msgs.back());
    result.gps->pose.pose.position.x = 0;
    result.gps->pose.pose.position.y = 0;
    result.gps->pose.pose.position.z = 0;

    for (const auto & m : gps_msgs) {
      result.gps->pose.pose.position.x += m->pose.pose.position.x;
      result.gps->pose.pose.position.y += m->pose.pose.position.y;
      result.gps->pose.pose.position.z += m->pose.pose.position.z;
    }
    double n_gps = gps_msgs.size();
    result.gps->pose.pose.position.x /= n_gps;
    result.gps->pose.pose.position.y /= n_gps;
    result.gps->pose.pose.position.z /= n_gps;
  }

  // Average Depth
  result.depth = std::make_shared<nav_msgs::msg::Odometry>(*depth_msgs.back());
  result.depth->pose.pose.position.z = 0;

  for (const auto & m : depth_msgs) {
    result.depth->pose.pose.position.z += m->pose.pose.position.z;
  }
  result.depth->pose.pose.position.z /= depth_msgs.size();

  // Average DVL
  result.dvl = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>(*dvl_msgs.back());
  result.dvl->twist.twist.linear.x = 0;
  result.dvl->twist.twist.linear.y = 0;
  result.dvl->twist.twist.linear.z = 0;

  for (const auto & m : dvl_msgs) {
    result.dvl->twist.twist.linear.x += m->twist.twist.linear.x;
    result.dvl->twist.twist.linear.y += m->twist.twist.linear.y;
    result.dvl->twist.twist.linear.z += m->twist.twist.linear.z;
  }
  double n_dvl = dvl_msgs.size();
  result.dvl->twist.twist.linear.x /= n_dvl;
  result.dvl->twist.twist.linear.y /= n_dvl;
  result.dvl->twist.twist.linear.z /= n_dvl;

  // Average Magnetometer
  if (mag_params_.enable) {
    result.mag = std::make_shared<sensor_msgs::msg::MagneticField>(*mag_msgs.back());
    result.mag->magnetic_field.x = 0;
    result.mag->magnetic_field.y = 0;
    result.mag->magnetic_field.z = 0;

    for (const auto & m : mag_msgs) {
      result.mag->magnetic_field.x += m->magnetic_field.x;
      result.mag->magnetic_field.y += m->magnetic_field.y;
      result.mag->magnetic_field.z += m->magnetic_field.z;
    }
    double n_mag = mag_msgs.size();
    result.mag->magnetic_field.x /= n_mag;
    result.mag->magnetic_field.y /= n_mag;
    result.mag->magnetic_field.z /= n_mag;
  }

  // Average AHRS
  if (ahrs_params_.enable_ahrs) {
    result.ahrs = std::make_shared<sensor_msgs::msg::Imu>(*ahrs_msgs.back());
    gtsam::Rot3 R_ref = toGtsam(ahrs_msgs.front()->orientation);
    gtsam::Vector3 log_sum = gtsam::Vector3::Zero();
    for (const auto & m : ahrs_msgs) {
      log_sum += gtsam::Rot3::Logmap(R_ref.between(toGtsam(m->orientation)));
    }
    gtsam::Vector3 log_avg = log_sum / static_cast<double>(ahrs_msgs.size());
    result.ahrs->orientation = toQuatMsg(R_ref.compose(gtsam::Rot3::Expmap(log_avg)));
  }

  return result;
}

gtsam::Rot3 FactorGraphNode::computeInitialOrientation()
{
  double roll = prior_params_.initial_orientation[0];
  double pitch = prior_params_.initial_orientation[1];
  double yaw = prior_params_.initial_orientation[2];
  gtsam::Rot3 R_base_dvl = toGtsam(dvl_to_base_tf_.transform.rotation);

  if (prior_params_.use_parameter_priors) {
    // Account for DVL rotation
    gtsam::Rot3 R_world_base = gtsam::Rot3::Ypr(yaw, pitch, roll);
    return R_world_base * R_base_dvl;
  }

  gtsam::Vector3 accel_imu(initial_imu_->linear_acceleration.x,
    initial_imu_->linear_acceleration.y,
    initial_imu_->linear_acceleration.z);

  // Account for IMU rotation
  gtsam::Pose3 T_dvl_imu = toGtsam(imu_to_dvl_tf_.transform);
  gtsam::Pose3 T_base_dvl = toGtsam(dvl_to_base_tf_.transform);
  gtsam::Vector3 accel_base = T_base_dvl.rotation() * (T_dvl_imu.rotation() * accel_imu);

  roll = std::atan2(accel_base.y(), accel_base.z());
  pitch = std::atan2(
    -accel_base.x(), std::sqrt(
      accel_base.y() * accel_base.y() +
      accel_base.z() * accel_base.z()));

  if (ahrs_params_.enable_ahrs) {
    // Account for AHRS sensor rotation
    gtsam::Rot3 R_dvl_sensor = toGtsam(ahrs_to_dvl_tf_.transform.rotation);
    gtsam::Rot3 R_base_sensor = R_base_dvl * R_dvl_sensor;
    gtsam::Rot3 R_world_sensor = toGtsam(initial_ahrs_->orientation);
    gtsam::Rot3 R_world_base_measured = R_world_sensor * R_base_sensor.inverse();
    yaw = R_world_base_measured.yaw();
  } else if (mag_params_.enable) {
    // Account for magnetometer rotation
    gtsam::Rot3 R_dvl_sensor = toGtsam(mag_to_dvl_tf_.transform.rotation);

    gtsam::Rot3 R_base_sensor = R_base_dvl * R_dvl_sensor;
    gtsam::Vector3 mag_sensor(initial_mag_->magnetic_field.x,
      initial_mag_->magnetic_field.y,
      initial_mag_->magnetic_field.z);
    gtsam::Vector3 mag_base = R_base_sensor * mag_sensor;

    // IMPORTANT! Use the tilt-compensated magnetic vector to calculate yaw
    gtsam::Rot3 R_rp = gtsam::Rot3::Ypr(0.0, pitch, roll);
    gtsam::Vector3 mag_horizontal = R_rp.unrotate(mag_base);

    double measured_yaw = std::atan2(mag_horizontal.y(), mag_horizontal.x());
    double ref_yaw = std::atan2(
      mag_params_.reference_field[1],
      mag_params_.reference_field[0]);

    yaw = measured_yaw - ref_yaw;
  }

  return gtsam::Rot3::Ypr(yaw, pitch, roll) * R_base_dvl;
}

gtsam::Point3 FactorGraphNode::computeInitialPosition(const gtsam::Rot3 & initial_orientation_dvl)
{
  gtsam::Point3 P_world_base(prior_params_.initial_position[0], prior_params_.initial_position[1],
    prior_params_.initial_position[2]);
  gtsam::Pose3 T_base_dvl = toGtsam(dvl_to_base_tf_.transform);
  gtsam::Rot3 R_world_base = initial_orientation_dvl * T_base_dvl.rotation().inverse();
  gtsam::Point3 P_world_dvl_param = P_world_base + R_world_base.rotate(T_base_dvl.translation());

  if (prior_params_.use_parameter_priors) {
    // Account for DVL lever arm
    return P_world_dvl_param;
  }

  gtsam::Point3 initial_position_dvl = P_world_dvl_param;
  if (gps_params_.enable) {
    // Account for GPS lever arm
    gtsam::Pose3 T_dvl_gps = toGtsam(gps_to_dvl_tf_.transform);
    gtsam::Point3 world_t_dvl_gps = initial_orientation_dvl.rotate(T_dvl_gps.translation());
    initial_position_dvl = toGtsam(initial_gps_->pose.pose.position) - world_t_dvl_gps;
  }
  gtsam::Pose3 T_dvl_depth = toGtsam(depth_to_dvl_tf_.transform);
  gtsam::Point3 world_t_dvl_depth = initial_orientation_dvl.rotate(T_dvl_depth.translation());
  initial_position_dvl.z() = initial_depth_->pose.pose.position.z - world_t_dvl_depth.z();

  return initial_position_dvl;
}

gtsam::Vector3 FactorGraphNode::computeInitialVelocity(const gtsam::Rot3 & initial_orientation_dvl)
{
  if (prior_params_.use_parameter_priors) {
    // Account for DVL rotation (in world frame)
    return initial_orientation_dvl.rotate(
      gtsam::Vector3(prior_params_.initial_velocity.data()));
  }

  // Account for DVL rotation (in world frame)
  return initial_orientation_dvl.rotate(toGtsam(initial_dvl_->twist.twist.linear));
}

gtsam::imuBias::ConstantBias FactorGraphNode::computeInitialBias()
{
  gtsam::Vector3 init_gyro_bias;
  if (prior_params_.use_parameter_priors) {
    init_gyro_bias =
      gtsam::Vector3(
      prior_params_.initial_gyro_bias[0], prior_params_.initial_gyro_bias[1],
      prior_params_.initial_gyro_bias[2]);
  } else {
    init_gyro_bias =
      gtsam::Vector3(
      initial_imu_->angular_velocity.x, initial_imu_->angular_velocity.y,
      initial_imu_->angular_velocity.z);
  }
  gtsam::Vector3 init_accel_bias(prior_params_.initial_accel_bias[0],
    prior_params_.initial_accel_bias[1],
    prior_params_.initial_accel_bias[2]);

  return gtsam::imuBias::ConstantBias(init_accel_bias, init_gyro_bias);
}

void FactorGraphNode::addPriorFactors(gtsam::NonlinearFactorGraph & graph, gtsam::Values & values)
{
  // Add initial pose prior
  gtsam::Vector6 prior_pose_sigmas;
  prior_pose_sigmas << prior_params_.initial_orientation_sigma,
    prior_params_.initial_orientation_sigma, prior_params_.initial_orientation_sigma,
    prior_params_.initial_position_sigma, prior_params_.initial_position_sigma,
    prior_params_.initial_position_sigma;

  if (!prior_params_.use_parameter_priors) {
    if (gps_params_.enable) {
      prior_pose_sigmas(3) = gps_params_.use_parameter_covariance ?
        gps_params_.position_noise_sigma :
        std::sqrt(initial_gps_->pose.covariance[0]);
      prior_pose_sigmas(4) = gps_params_.use_parameter_covariance ?
        gps_params_.position_noise_sigma :
        std::sqrt(initial_gps_->pose.covariance[7]);
    }
    prior_pose_sigmas(5) = depth_params_.use_parameter_covariance ?
      depth_params_.position_z_noise_sigma :
      std::sqrt(initial_depth_->pose.covariance[14]);

    if (ahrs_params_.enable_ahrs) {
      prior_pose_sigmas(2) = ahrs_params_.use_parameter_covariance ?
        ahrs_params_.yaw_noise_sigma :
        std::sqrt(initial_ahrs_->orientation_covariance[8]);
    } else if (mag_params_.enable) {
      prior_pose_sigmas(2) = mag_params_.use_parameter_covariance ?
        mag_params_.magnetic_field_noise_sigma :
        std::sqrt(initial_mag_->magnetic_field_covariance[8]);
    }
    prior_pose_sigmas(0) = imu_params_.use_parameter_covariance ?
      imu_params_.gyro_noise_sigma :
      initial_imu_->angular_velocity_covariance[0];
    prior_pose_sigmas(1) = imu_params_.use_parameter_covariance ?
      imu_params_.gyro_noise_sigma :
      initial_imu_->angular_velocity_covariance[4];
  }

  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    X(0), prev_pose_, gtsam::noiseModel::Diagonal::Sigmas(prior_pose_sigmas));
  values.insert(X(0), prev_pose_);

  // Add initial velocity prior
  gtsam::SharedNoiseModel prior_vel_noise;
  if (prior_params_.use_parameter_priors) {
    prior_vel_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, prior_params_.initial_velocity_sigma);
  } else {
    if (dvl_params_.use_parameter_covariance) {
      prior_vel_noise =
        gtsam::noiseModel::Isotropic::Sigma(3, dvl_params_.velocity_noise_sigma);
    } else {
      gtsam::Vector3 prior_dvl_sigmas(sqrt(initial_dvl_->twist.covariance[0]),
        sqrt(initial_dvl_->twist.covariance[7]),
        sqrt(initial_dvl_->twist.covariance[14]));
      prior_vel_noise = gtsam::noiseModel::Diagonal::Sigmas(prior_dvl_sigmas);
    }
  }

  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), prev_vel_, prior_vel_noise);
  values.insert(V(0), prev_vel_);

  // Add initial IMU bias prior
  gtsam::Vector6 prior_imu_bias_sigmas;
  prior_imu_bias_sigmas << gtsam::Vector3::Constant(prior_params_.initial_accel_bias_sigma),
    gtsam::Vector3::Constant(prior_params_.initial_gyro_bias_sigma);
  gtsam::SharedNoiseModel prior_imu_bias_noise =
    gtsam::noiseModel::Diagonal::Sigmas(prior_imu_bias_sigmas);

  graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
    B(0), prev_imu_bias_,
    prior_imu_bias_noise);
  values.insert(B(0), prev_imu_bias_);
}

void FactorGraphNode::initializeGraph()
{
  // --- Wait for Sensor Data ---
  if (!sensors_ready_) {
    std::scoped_lock lock(imu_queue_mutex_, gps_queue_mutex_, depth_queue_mutex_,
      mag_queue_mutex_, ahrs_queue_mutex_, dvl_queue_mutex_);

    if (!imu_queue_.empty() && (!gps_params_.enable || !gps_queue_.empty()) &&
      !depth_queue_.empty() && (!mag_params_.enable || !mag_queue_.empty()) &&
      (!ahrs_params_.enable_ahrs || !ahrs_queue_.empty()) && !dvl_queue_.empty())
    {
      if (prior_params_.use_parameter_priors) {
        RCLCPP_INFO(
          get_logger(),
          "Required sensor messages received! Skipping averaging...");
        initial_imu_ = imu_queue_.back();
        if (gps_params_.enable) {initial_gps_ = gps_queue_.back();}
        initial_depth_ = depth_queue_.back();
        if (mag_params_.enable) {initial_mag_ = mag_queue_.back();}
        if (ahrs_params_.enable_ahrs) {initial_ahrs_ = ahrs_queue_.back();}
        initial_dvl_ = dvl_queue_.back();
        data_averaged_ = true;
      } else {
        RCLCPP_INFO(get_logger(), "Required sensor messages received! Averaging...");
        start_avg_time_ = this->get_clock()->now().seconds();
      }
      sensors_ready_ = true;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for sensors: %s%s%s%s%s%s",
        imu_queue_.empty() ? "[IMU] " : "",
        (gps_params_.enable && gps_queue_.empty()) ? "[GPS] " : "",
        (mag_params_.enable && mag_queue_.empty()) ? "[Magnetometer] " : "",
        (ahrs_params_.enable_ahrs && ahrs_queue_.empty()) ? "[AHRS] " : "",
        depth_queue_.empty() ? "[Depth] " : "", dvl_queue_.empty() ? "[DVL]" : "");
      return;
    }
  }

  // --- Average Initial Measurements ---
  if (!data_averaged_) {
    double duration = this->get_clock()->now().seconds() - start_avg_time_;
    if (duration < prior_params_.initialization_duration) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Averaging sensor data (%.2fs / %.2fs)...", duration,
        prior_params_.initialization_duration);
      return;
    } else {
      std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_msgs;
      std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_msgs;
      std::deque<nav_msgs::msg::Odometry::SharedPtr> depth_msgs;
      std::deque<sensor_msgs::msg::MagneticField::SharedPtr> mag_msgs;
      std::deque<sensor_msgs::msg::Imu::SharedPtr> ahrs_msgs;
      std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> dvl_msgs;

      {
        std::scoped_lock lock(imu_queue_mutex_, gps_queue_mutex_, depth_queue_mutex_,
          mag_queue_mutex_, ahrs_queue_mutex_, dvl_queue_mutex_);

        imu_msgs = std::move(imu_queue_);
        gps_msgs = std::move(gps_queue_);
        depth_msgs = std::move(depth_queue_);
        mag_msgs = std::move(mag_queue_);
        ahrs_msgs = std::move(ahrs_queue_);
        dvl_msgs = std::move(dvl_queue_);
      }

      AveragedMeasurements avgs =
        computeAveragedMeasurements(
        imu_msgs, gps_msgs, depth_msgs, mag_msgs, ahrs_msgs,
        dvl_msgs);

      initial_imu_ = avgs.imu;
      initial_gps_ = avgs.gps;
      initial_depth_ = avgs.depth;
      initial_mag_ = avgs.mag;
      initial_ahrs_ = avgs.ahrs;
      initial_dvl_ = avgs.dvl;

      RCLCPP_INFO(get_logger(), "Sensor data averaged successfully! Initializing graph...");
      data_averaged_ = true;
    }
  }

  if (!lookupInitialTransforms()) {return;}

  // --- Create Initial Factor Graph ---
  gtsam::NonlinearFactorGraph initial_graph;
  gtsam::Values initial_values;

  gtsam::Rot3 initial_orientation_dvl = computeInitialOrientation();
  gtsam::Point3 initial_position_dvl = computeInitialPosition(initial_orientation_dvl);
  prev_pose_ = gtsam::Pose3(initial_orientation_dvl, initial_position_dvl);
  prev_vel_ = computeInitialVelocity(initial_orientation_dvl);
  prev_imu_bias_ = computeInitialBias();

  addPriorFactors(initial_graph, initial_values);

  rclcpp::Time init_stamp = experimental_params_.enable_dvl_preintegration ?
    initial_depth_->header.stamp :
    initial_dvl_->header.stamp;
  prev_time_ = init_stamp.seconds();
  time_to_key_[init_stamp] = X(0);

  // --- Initialize Preintegrators ---
  imu_preintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
    configureImuPreintegration(), prev_imu_bias_);
  if (experimental_params_.enable_dvl_preintegration) {
    dvl_preintegrator_ = std::make_unique<DVLPreintegrator>();
    dvl_preintegrator_->reset(initial_orientation_dvl);
  }

  // --- Initialize Incremental Fixed-Lag Smoother ---
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap initial_timestamps;
  initial_timestamps[X(0)] = prev_time_;
  initial_timestamps[V(0)] = prev_time_;
  initial_timestamps[B(0)] = prev_time_;

  gtsam::ISAM2Params isam2_params;
  isam2_params.relinearizeThreshold = gtsam_relinearize_threshold_;
  isam2_params.relinearizeSkip = gtsam_relinearize_skip_;
  smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(smoother_lag_, isam2_params);
  smoother_->update(initial_graph, initial_values, initial_timestamps);

  graph_initialized_ = true;
  RCLCPP_INFO(get_logger(), "Graph initialized successfully!");
}

void FactorGraphNode::addGpsFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<nav_msgs::msg::Odometry::SharedPtr> & gps_msgs)
{
  if (!have_gps_to_dvl_tf_ || gps_msgs.empty()) {return;}

  const auto & gps_msg = gps_msgs.back();

  gtsam::Vector3 gps_sigmas;
  if (gps_params_.use_parameter_covariance) {
    gps_sigmas << gps_params_.position_noise_sigma, gps_params_.position_noise_sigma,
      gps_params_.altitude_noise_sigma;
  } else {
    gps_sigmas << sqrt(gps_msg->pose.covariance[0]), sqrt(gps_msg->pose.covariance[7]),
      gps_params_.altitude_noise_sigma;        // Enable ignoring altitude from GPS
  }
  gtsam::SharedNoiseModel gps_noise = gtsam::noiseModel::Diagonal::Sigmas(gps_sigmas);

  if (gps_params_.robust_kernel == "Huber") {
    gps_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(gps_params_.robust_k), gps_noise);
  } else if (gps_params_.robust_kernel == "Tukey") {
    gps_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Tukey::Create(gps_params_.robust_k), gps_noise);
  }

  graph.emplace_shared<CustomGPSFactorArm>(
    X(current_step_), toGtsam(gps_msg->pose.pose.position),
    toGtsam(gps_to_dvl_tf_.transform), gps_noise);
}

void FactorGraphNode::addDepthFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<nav_msgs::msg::Odometry::SharedPtr> & depth_msgs)
{
  if (!have_depth_to_dvl_tf_ || depth_msgs.empty()) {return;}

  const auto & depth_msg = depth_msgs.back();

  double depth_sigma;
  if (depth_params_.use_parameter_covariance) {
    depth_sigma = depth_params_.position_z_noise_sigma;
  } else {
    depth_sigma = sqrt(depth_msg->pose.covariance[14]);
  }
  gtsam::SharedNoiseModel depth_noise = gtsam::noiseModel::Isotropic::Sigma(1, depth_sigma);

  if (depth_params_.robust_kernel == "Huber") {
    depth_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(depth_params_.robust_k), depth_noise);
  } else if (depth_params_.robust_kernel == "Tukey") {
    depth_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Tukey::Create(depth_params_.robust_k), depth_noise);
  }

  graph.emplace_shared<CustomDepthFactorArm>(
    X(current_step_), depth_msg->pose.pose.position.z,
    toGtsam(depth_to_dvl_tf_.transform), depth_noise);
}

void FactorGraphNode::addAhrsFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<sensor_msgs::msg::Imu::SharedPtr> & ahrs_msgs)
{
  if (!have_ahrs_to_dvl_tf_ || ahrs_msgs.empty()) {return;}

  const auto & ahrs_msg = ahrs_msgs.back();

  gtsam::Vector3 ahrs_sigmas;
  if (ahrs_params_.use_parameter_covariance) {
    ahrs_sigmas << ahrs_params_.roll_pitch_noise_sigma,
      ahrs_params_.roll_pitch_noise_sigma, ahrs_params_.yaw_noise_sigma;
  } else {
    ahrs_sigmas
      << ahrs_params_.roll_pitch_noise_sigma,        // Enable ignoring roll from AHRS sensor
      ahrs_params_.roll_pitch_noise_sigma,           // Enable ignoring pitch from AHRS sensor
      sqrt(ahrs_msg->orientation_covariance[8]);
  }
  gtsam::SharedNoiseModel ahrs_noise = gtsam::noiseModel::Diagonal::Sigmas(ahrs_sigmas);

  if (ahrs_params_.robust_kernel == "Huber") {
    ahrs_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(ahrs_params_.robust_k), ahrs_noise);
  } else if (ahrs_params_.robust_kernel == "Tukey") {
    ahrs_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Tukey::Create(ahrs_params_.robust_k), ahrs_noise);
  }

  graph.emplace_shared<AhrsFactor>(
    X(current_step_), toGtsam(ahrs_msg->orientation),
    toGtsam(ahrs_to_dvl_tf_.transform.rotation), ahrs_noise);
}

void FactorGraphNode::addMagFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<sensor_msgs::msg::MagneticField::SharedPtr> & mag_msgs)
{
  if (!have_mag_to_dvl_tf_ || mag_msgs.empty()) {return;}

  const auto & mag_msg = mag_msgs.back();

  gtsam::Point3 ref_vec(mag_params_.reference_field[0],
    mag_params_.reference_field[1],
    mag_params_.reference_field[2]);

  if (mag_params_.constrain_yaw_only) {
    // 1D Factor
    gtsam::Vector1 mag_sigma;
    if (mag_params_.use_parameter_covariance) {
      mag_sigma << mag_params_.magnetic_field_noise_sigma;
    } else {
      double B_mag = ref_vec.norm();
      if (B_mag > 1e-6) {
        // IMPORTANT! Convert to angular uncertainty (rad) from magnetic field uncertainty
        double sigma_B = sqrt(mag_msg->magnetic_field_covariance[0]);
        mag_sigma << sigma_B / B_mag;
      } else {
        mag_sigma << mag_params_.magnetic_field_noise_sigma;
      }
    }
    gtsam::SharedNoiseModel mag_noise = gtsam::noiseModel::Isotropic::Sigma(1, mag_sigma(0));

    if (mag_params_.robust_kernel == "Huber") {
      mag_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(mag_params_.robust_k), mag_noise);
    } else if (mag_params_.robust_kernel == "Tukey") {
      mag_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Tukey::Create(mag_params_.robust_k), mag_noise);
    }

    graph.emplace_shared<CustomMagFactorArm>(
      X(current_step_), toGtsam(mag_msg->magnetic_field), ref_vec,
      toGtsam(mag_to_dvl_tf_.transform.rotation), mag_noise, true);

  } else {
    // Standard 3D Factor
    gtsam::Vector3 mag_sigmas;
    if (mag_params_.use_parameter_covariance) {
      mag_sigmas << mag_params_.magnetic_field_noise_sigma, mag_params_.magnetic_field_noise_sigma,
        mag_params_.magnetic_field_noise_sigma;
    } else {
      mag_sigmas << sqrt(mag_msg->magnetic_field_covariance[0]),
        sqrt(mag_msg->magnetic_field_covariance[4]),
        sqrt(mag_msg->magnetic_field_covariance[8]);
    }
    gtsam::SharedNoiseModel mag_noise = gtsam::noiseModel::Diagonal::Sigmas(mag_sigmas);

    if (mag_params_.robust_kernel == "Huber") {
      mag_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(mag_params_.robust_k), mag_noise);
    } else if (mag_params_.robust_kernel == "Tukey") {
      mag_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Tukey::Create(mag_params_.robust_k), mag_noise);
    }

    graph.emplace_shared<CustomMagFactorArm>(
      X(current_step_), toGtsam(mag_msg->magnetic_field), ref_vec,
      toGtsam(mag_to_dvl_tf_.transform.rotation), mag_noise, false);
  }
}

void FactorGraphNode::addDvlFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & dvl_msgs)
{
  if (dvl_msgs.empty()) {return;}

  const auto & dvl_msg = dvl_msgs.back();

  gtsam::Vector3 dvl_sigmas;
  if (dvl_params_.use_parameter_covariance) {
    dvl_sigmas << dvl_params_.velocity_noise_sigma, dvl_params_.velocity_noise_sigma,
      dvl_params_.velocity_noise_sigma;
  } else {
    dvl_sigmas << sqrt(dvl_msg->twist.covariance[0]), sqrt(dvl_msg->twist.covariance[7]),
      sqrt(dvl_msg->twist.covariance[14]);
  }
  gtsam::SharedNoiseModel dvl_noise = gtsam::noiseModel::Diagonal::Sigmas(dvl_sigmas);

  if (dvl_params_.robust_kernel == "Huber") {
    dvl_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(dvl_params_.robust_k), dvl_noise);
  } else if (dvl_params_.robust_kernel == "Tukey") {
    dvl_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Tukey::Create(dvl_params_.robust_k), dvl_noise);
  }

  graph.emplace_shared<CustomDVLFactor>(
    X(current_step_), V(current_step_),
    toGtsam(dvl_msg->twist.twist.linear), dvl_noise);
}

void FactorGraphNode::addPreintegratedImuFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs, double target_time)
{
  if (!have_imu_to_dvl_tf_ || !imu_preintegrator_) {return;}

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

    double dt = current_imu_time - last_imu_time;
    if (dt > 1e-9) {
      last_acc = toGtsam(imu_msg->linear_acceleration);
      last_gyr = toGtsam(imu_msg->angular_velocity);
      imu_preintegrator_->integrateMeasurement(last_acc, last_gyr, dt);
    }
    last_imu_time = current_imu_time;
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

  // Re-queue future IMU messages
  if (!unused_imu_msgs.empty()) {
    std::scoped_lock lock(imu_queue_mutex_);
    imu_queue_.insert(imu_queue_.begin(), unused_imu_msgs.begin(), unused_imu_msgs.end());
  }
}

gtsam::Rot3 FactorGraphNode::getInterpolatedOrientation(
  const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs, double target_time)
{
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
  double alpha = (target_time - t1) / (t2 - t1);

  // Use Slerp for quaternion interpolation (handles alpha > 1.0 for extrapolation)
  return toGtsam((*(it_after - 1))->orientation).slerp(alpha, toGtsam((*it_after)->orientation));
}

void FactorGraphNode::addPreintegratedDvlFactor(
  gtsam::NonlinearFactorGraph & graph,
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & dvl_msgs,
  const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs, double target_time)
{
  if (!have_imu_to_dvl_tf_ || !dvl_preintegrator_ || dvl_msgs.empty()) {return;}

  if (imu_msgs.empty()) {
    std::scoped_lock lock(dvl_queue_mutex_);
    dvl_queue_.insert(dvl_queue_.begin(), dvl_msgs.begin(), dvl_msgs.end());
    return;
  }

  double last_dvl_time = prev_time_;
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> unused_dvl_msgs;

  gtsam::Vector3 last_linear_vel = gtsam::Vector3::Zero();
  gtsam::Matrix3 last_covariance = gtsam::Matrix3::Zero();

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

    double dt = current_dvl_time - last_dvl_time;
    if (dt > 1e-9) {
      last_linear_vel = toGtsam(dvl_msg->twist.twist.linear);

      if (dvl_params_.use_parameter_covariance) {
        last_covariance = gtsam::Matrix3::Identity() * (dvl_params_.velocity_noise_sigma *
          dvl_params_.velocity_noise_sigma);
      } else {
        last_covariance.setZero();
        last_covariance(0, 0) = dvl_msg->twist.covariance[0];
        last_covariance(1, 1) = dvl_msg->twist.covariance[7];
        last_covariance(2, 2) = dvl_msg->twist.covariance[14];
      }

      // Integrate DVL measurement alongside interpolated IMU attitude
      gtsam::Rot3 cur_imu_att = getInterpolatedOrientation(imu_msgs, current_dvl_time);
      gtsam::Rot3 cur_dvl_att = cur_imu_att * R_imu_to_dvl;

      dvl_preintegrator_->integrateMeasurement(
        last_linear_vel, cur_dvl_att, dt,
        last_covariance);
    }
    last_dvl_time = current_dvl_time;
  }

  // Extra measurement to reach exact target time
  if (last_dvl_time < target_time) {
    double dt = target_time - last_dvl_time;
    if (dt > 1e-6) {
      gtsam::Rot3 cur_imu_att = getInterpolatedOrientation(imu_msgs, target_time);
      gtsam::Rot3 cur_dvl_att = cur_imu_att * R_imu_to_dvl;
      dvl_preintegrator_->integrateMeasurement(
        last_linear_vel, cur_dvl_att, dt,
        last_covariance);
    }
    last_dvl_time = target_time;
  }

  graph.emplace_shared<CustomDVLPreintegratedFactor>(
    X(prev_step_), X(current_step_), dvl_preintegrator_->delta(),
    gtsam::noiseModel::Gaussian::Covariance(dvl_preintegrator_->covariance()));

  // Re-queue future DVL messages
  if (!unused_dvl_msgs.empty()) {
    std::scoped_lock lock(dvl_queue_mutex_);
    dvl_queue_.insert(dvl_queue_.begin(), unused_dvl_msgs.begin(), unused_dvl_msgs.end());
  }
}

void FactorGraphNode::publishGlobalOdom(
  const gtsam::Pose3 & current_pose,
  const gtsam::Matrix & pose_covariance,
  const rclcpp::Time & timestamp)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = timestamp;
  odom_msg.header.frame_id = map_frame_;
  odom_msg.child_frame_id = base_frame_;
  odom_msg.pose.pose = toPoseMsg(current_pose);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom_msg.pose.covariance[i * 6 + j] = pose_covariance(i + 3, j + 3);
      odom_msg.pose.covariance[(i + 3) * 6 + (j + 3)] = pose_covariance(i, j);
      odom_msg.pose.covariance[i * 6 + (j + 3)] = pose_covariance(i + 3, j);
      odom_msg.pose.covariance[(i + 3) * 6 + j] = pose_covariance(i, j + 3);
    }
  }
  odom_msg.twist.covariance[0] = -1.0;
  global_odom_pub_->publish(odom_msg);
}

void FactorGraphNode::broadcastGlobalTf(
  const gtsam::Pose3 & current_pose,
  const rclcpp::Time & timestamp)
{
  try {
    gtsam::Pose3 T_odom_base = toGtsam(
      tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero).transform);
    gtsam::Pose3 T_map_odom = current_pose * T_odom_base.inverse();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
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
  path_msg.header.frame_id = map_frame_;

  gtsam::Pose3 T_dvl_base = toGtsam(dvl_to_base_tf_.transform).inverse();

  for (const auto & pair : time_to_key_) {
    if (results.exists(pair.second)) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = map_frame_;
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

  // IMPORTANT! This is the velocity at the DVL with respect to the 'map_frame_'.
  vel_msg.header.frame_id = map_frame_;
  vel_msg.twist.twist.linear = toVectorMsg(current_vel);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      vel_msg.twist.covariance[i * 6 + j] = vel_covariance(i, j);
    }
  }

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

  // GTSAM IMU bias covariance is [ accel_3x3 | ... ]
  //                          [ ...       | gyro_3x3  ]
  // ROS twist covariance is  [ linear_3x3 | ... ]
  //                          [ ...        | angular_3x3 ]
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      imu_bias_msg.twist.covariance[i * 6 + j] = imu_bias_covariance(i, j);
    }
  }

  imu_bias_pub_->publish(imu_bias_msg);
}

void FactorGraphNode::optimizeGraph()
{
  // --- Handle Processing Overflow ---
  std::unique_lock<std::mutex> opt_lock(optimization_mutex_, std::try_to_lock);
  if (!opt_lock.owns_lock()) {return;}

  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_msgs;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_msgs;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> depth_msgs;
  std::deque<sensor_msgs::msg::MagneticField::SharedPtr> mag_msgs;
  std::deque<sensor_msgs::msg::Imu::SharedPtr> ahrs_msgs;
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> dvl_msgs;

  {
    std::scoped_lock lock(imu_queue_mutex_, gps_queue_mutex_, depth_queue_mutex_,
      mag_queue_mutex_, ahrs_queue_mutex_, dvl_queue_mutex_);

    imu_msgs = std::move(imu_queue_);
    gps_msgs = std::move(gps_queue_);
    depth_msgs = std::move(depth_queue_);
    mag_msgs = std::move(mag_queue_);
    ahrs_msgs = std::move(ahrs_queue_);
    dvl_msgs = std::move(dvl_queue_);
  }

  if (experimental_params_.enable_dvl_preintegration) {
    if (depth_msgs.size() > 1) {
      RCLCPP_WARN(
        get_logger(), "Processing overflow. Skipping %ld Depth keyframes.",
        depth_msgs.size() - 1);
    }
  } else {
    if (dvl_msgs.size() > 1) {
      RCLCPP_WARN(
        get_logger(), "Processing overflow. Skipping %ld DVL keyframes.",
        dvl_msgs.size() - 1);
    }
  }

  // Sort IMU messages -- should already be in order, but just in case
  auto by_time = [](const auto & a, const auto & b) {
      return rclcpp::Time(a->header.stamp) < rclcpp::Time(b->header.stamp);
    };
  std::sort(imu_msgs.begin(), imu_msgs.end(), by_time);

  rclcpp::Time target_stamp;
  if (experimental_params_.enable_dvl_preintegration) {
    target_stamp = depth_msgs.back()->header.stamp;
  } else {
    if (!dvl_msgs.empty()) {
      target_stamp = dvl_msgs.back()->header.stamp;
    } else {
      target_stamp = depth_msgs.back()->header.stamp;
    }
  }
  double target_time = target_stamp.seconds();

  // --- Create New Factor Graph ---
  gtsam::NonlinearFactorGraph new_graph;
  gtsam::Values new_values;
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap new_timestamps;

  addPreintegratedImuFactor(new_graph, imu_msgs, target_time);
  if (gps_params_.enable) {addGpsFactor(new_graph, gps_msgs);}
  addDepthFactor(new_graph, depth_msgs);
  if (mag_params_.enable) {addMagFactor(new_graph, mag_msgs);}
  if (ahrs_params_.enable_ahrs) {addAhrsFactor(new_graph, ahrs_msgs);}

  if (experimental_params_.enable_dvl_preintegration) {
    addPreintegratedDvlFactor(new_graph, dvl_msgs, imu_msgs, target_time);
  } else {
    addDvlFactor(new_graph, dvl_msgs);
  }

  // --- Incremental Fixed-Lag Smoother Update ---
  auto pred = imu_preintegrator_->predict(gtsam::NavState(prev_pose_, prev_vel_), prev_imu_bias_);
  new_values.insert(X(current_step_), pred.pose());
  new_values.insert(V(current_step_), pred.velocity());
  new_values.insert(B(current_step_), prev_imu_bias_);
  new_timestamps[X(current_step_)] = target_time;
  new_timestamps[V(current_step_)] = target_time;
  new_timestamps[B(current_step_)] = target_time;

  try {
    smoother_->update(new_graph, new_values, new_timestamps);
    prev_pose_ = smoother_->calculateEstimate<gtsam::Pose3>(X(current_step_));
    gtsam::Matrix new_pose_cov = publish_pose_cov_ ?
      smoother_->marginalCovariance(X(current_step_)) :
      gtsam::Matrix::Identity(6, 6) * -1.0;

    prev_vel_ = smoother_->calculateEstimate<gtsam::Vector3>(V(current_step_));
    prev_imu_bias_ =
      smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(current_step_));
    imu_preintegrator_->resetIntegrationAndSetBias(prev_imu_bias_);

    time_to_key_[target_stamp] = X(current_step_);
    time_to_key_.erase(
      time_to_key_.begin(),
      time_to_key_.lower_bound(target_stamp - rclcpp::Duration::from_seconds(smoother_lag_)));

    // --- Publish Global Odometry ---
    gtsam::Pose3 T_base_dvl = toGtsam(dvl_to_base_tf_.transform);
    publishGlobalOdom(prev_pose_ * T_base_dvl.inverse(), new_pose_cov, target_stamp);

    // --- Publish Global TF ---
    if (publish_global_tf_) {broadcastGlobalTf(prev_pose_ * T_base_dvl.inverse(), target_stamp);}

    // --- Publish Smoothed Path ---
    if (publish_smoothed_path_) {
      publishSmoothedPath(smoother_->calculateEstimate(), target_stamp);
    }

    // --- Publish Velocity ---
    if (publish_velocity_) {
      publishVelocity(
        prev_vel_, smoother_->marginalCovariance(V(current_step_)),
        target_stamp);
    }

    // --- Publish IMU Bias ---
    if (publish_imu_bias_) {
      publishImuBias(
        prev_imu_bias_, smoother_->marginalCovariance(B(current_step_)),
        target_stamp);
    }

    prev_time_ = target_time;
    prev_step_ = current_step_;
    current_step_++;
  } catch (const gtsam::IndeterminantLinearSystemException & e) {
    RCLCPP_FATAL(get_logger(), "GTSAM IndeterminantLinearSystemException: %s", e.what());
    rclcpp::shutdown();
  }
}

}  // namespace coug_fgo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<coug_fgo::FactorGraphNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
