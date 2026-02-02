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
 * @file factor_graph_node.hpp
 * @brief ROS 2 node for factor graph optimization using GTSAM and fixed-lag smoothing.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "coug_fgo/utils/conversion_utils.hpp"
#include "coug_fgo/utils/dvl_preintegrator.hpp"
#include <coug_fgo/factor_graph_parameters.hpp>


namespace coug_fgo
{

/**
 * @class FactorGraphNode
 * @brief Performs factor graph optimization for AUV navigation.
 *
 * This node integrates measurements from IMU, DVL, GPS, depth, magnetometer, and AHRS
 * sensors into a global factor graph. It uses fixed-lag smoothing (ISAM2) to estimate
 * the AUV's pose, velocity, and IMU biases in real-time.
 */
class FactorGraphNode : public rclcpp::Node
{
public:
  /**
   * @brief FactorGraphNode constructor.
   */
  FactorGraphNode();

  struct AveragedMeasurements
  {
    sensor_msgs::msg::Imu::SharedPtr imu;
    nav_msgs::msg::Odometry::SharedPtr gps;
    nav_msgs::msg::Odometry::SharedPtr depth;
    sensor_msgs::msg::Imu::SharedPtr ahrs;
    sensor_msgs::msg::MagneticField::SharedPtr mag;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr dvl;
  };

private:
  // --- Main Logic ---
  /**
   * @brief Initializes the factor graph using averaged sensor data or parameters.
   */
  void initializeGraph();

  /**
   * @brief Main optimization loop; adds factors to the graph and invokes the smoother.
   */
  void optimizeGraph();

  // --- Setup & Helpers ---
  /**
   * @brief Initializes publishers, subscribers, and timers.
   */
  void setupRosInterfaces();
  /**
   * @brief Looks up required static transforms (e.g., imu->dvl).
   * @return True if all transforms were found.
   */
  bool lookupInitialTransforms();
  /**
   * @brief Configures GTSAM combined IMU preintegration parameters.
   * @return GTSAM preintegration parameters.
   */
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
  configureImuPreintegration();

  /**
   * @brief Averages sensor queues over a specified duration.
   * @param imu_msgs Queue of IMU messages.
   * @param gps_msgs Queue of GPS messages.
   * @param depth_msgs Queue of depth messages.
   * @param mag_msgs Queue of magnetic field messages.
   * @param ahrs_msgs Queue of AHRS messages.
   * @param dvl_msgs Queue of DVL messages.
   * @return The averaged measurements.
   */
  AveragedMeasurements computeAveragedMeasurements(
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs,
    const std::deque<nav_msgs::msg::Odometry::SharedPtr> & gps_msgs,
    const std::deque<nav_msgs::msg::Odometry::SharedPtr> & depth_msgs,
    const std::deque<sensor_msgs::msg::MagneticField::SharedPtr> & mag_msgs,
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & ahrs_msgs,
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & dvl_msgs);

  /**
   * @brief Computes initial orientation using IMU, magnetometer, and AHRS sensor.
   * @return The initial GTSAM rotation.
   */
  gtsam::Rot3 computeInitialOrientation();

  /**
   * @brief Computes initial position using GPS and depth sensor.
   * @param initial_orientation The previously computed initial orientation.
   * @return The initial GTSAM translation.
   */
  gtsam::Point3 computeInitialPosition(const gtsam::Rot3 & initial_orientation);

  /**
   * @brief Computes initial velocity using DVL measurements.
   * @param initial_orientation The previously computed initial orientation.
   * @return The initial GTSAM velocity.
   */
  gtsam::Vector3 computeInitialVelocity(const gtsam::Rot3 & initial_orientation);

  /**
   * @brief Computes initial IMU biases from averaged data or parameters.
   * @return The initial GTSAM bias estimate.
   */
  gtsam::imuBias::ConstantBias computeInitialBias();

  // --- Factor Management ---
  /**
   * @brief Adds prior factors to the GTSAM graph.
   * @param graph The target factor graph.
   * @param values The initial value estimates.
   */
  void addPriorFactors(gtsam::NonlinearFactorGraph & graph, gtsam::Values & values);

  /**
   * @brief Adds a GPS position factor to the graph.
   * @param graph The target factor graph.
   * @param gps_msgs Queue of GPS messages to process.
   */
  void addGpsFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<nav_msgs::msg::Odometry::SharedPtr> & gps_msgs);

  /**
   * @brief Adds a depth (Z) factor to the graph.
   * @param graph The target factor graph.
   * @param depth_msgs Queue of depth messages to process.
   */
  void addDepthFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<nav_msgs::msg::Odometry::SharedPtr> & depth_msgs);

  /**
   * @brief Adds a magnetic orientation factor to the graph.
   * @param graph The target factor graph.
   * @param mag_msgs Queue of Mag messages.
   */
  void addMagFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<sensor_msgs::msg::MagneticField::SharedPtr> & mag_msgs);

  /**
   * @brief Adds a AHRS orientation factor to the graph.
   * @param graph The target factor graph.
   * @param ahrs_msgs Queue of AHRS messages to process.
   */
  void addAhrsFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & ahrs_msgs);

  /**
   * @brief Adds a velocity (DVL) factor to the graph.
   * @param graph The target factor graph.
   * @param dvl_msgs Queue of DVL messages to process.
   */
  void addDvlFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & dvl_msgs,
    double target_time);

  /**
   * @brief Integrates and adds a combined IMU factor to the graph.
   * @param graph The target factor graph.
   * @param imu_msgs Queue of IMU messages since the last pose.
   * @param target_time The timestamp for the new pose key.
   */
  void addPreintegratedImuFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs,
    double target_time);

  /**
   * @brief Interpolates orientation between IMU messages.
   * @param imu_msgs IMU message history.
   * @param target_time Integration target time.
   * @return The interpolated GTSAM rotation.
   */
  gtsam::Rot3 getInterpolatedOrientation(
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs, double target_time);

  /**
   * @brief [EXPERIMENTAL] Adds a preintegrated DVL translation factor.
   * @param graph The target factor graph.
   * @param dvl_msgs Queue of DVL measurements.
   * @param imu_msgs Queue of IMU measurements for orientation.
   * @param target_time The timestamp for the new pose key.
   */
  void addPreintegratedDvlFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & dvl_msgs,
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs, double target_time);

  // --- Publishing ---
  /**
   * @brief Publishes the optimized global odometry.
   * @param current_pose The estimated pose.
   * @param pose_covariance The estimation error covariance.
   * @param timestamp The message timestamp.
   */
  void publishGlobalOdom(
    const gtsam::Pose3 & current_pose, const gtsam::Matrix & pose_covariance,
    const rclcpp::Time & timestamp);

  /**
   * @brief Broadcasts the map-to-odom transform.
   * @param current_pose The estimated pose.
   * @param timestamp The transform timestamp.
   */
  void broadcastGlobalTf(const gtsam::Pose3 & current_pose, const rclcpp::Time & timestamp);

  /**
   * @brief Publishes the full optimized trajectory path.
   * @param results The final optimized values.
   * @param timestamp The path timestamp.
   */
  void publishSmoothedPath(const gtsam::Values & results, const rclcpp::Time & timestamp);

  /**
   * @brief Publishes the optimized body-frame velocity.
   * @param current_vel The estimated velocity.
   * @param vel_covariance The estimation error covariance.
   * @param timestamp The message timestamp.
   */
  void publishVelocity(
    const gtsam::Vector3 & current_vel, const gtsam::Matrix & vel_covariance,
    const rclcpp::Time & timestamp);

  /**
   * @brief Publishes the optimized IMU biases.
   * @param current_imu_bias The estimated biases.
   * @param imu_bias_covariance The estimation error covariance.
   * @param timestamp The message timestamp.
   */
  void publishImuBias(
    const gtsam::imuBias::ConstantBias & current_imu_bias,
    const gtsam::Matrix & imu_bias_covariance, const rclcpp::Time & timestamp);

  // --- Diagnostics ---
  /**
   * @brief Checks sensor inputs for queue sizes and data freshness.
   * @param stat The diagnostic status wrapper.
   */
  void checkSensorInputs(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Checks the overall graph lifecycle state.
   * @param stat The diagnostic status wrapper.
   */
  void checkGraphState(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Checks for performance issues like processing overflow.
   * @param stat The diagnostic status wrapper.
   */
  void checkProcessingOverflow(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // --- Graph State ---
  bool sensors_ready_ = false;
  bool data_averaged_ = false;
  std::atomic<bool> graph_initialized_{false};
  double start_avg_time_ = 0.0;

  size_t prev_step_ = 0;
  size_t current_step_ = 1;
  double prev_time_ = 0.0;
  std::atomic<double> last_dvl_time_{0.0};
  std::atomic<double> last_imu_time_{0.0};
  std::atomic<double> last_gps_time_{0.0};
  std::atomic<double> last_depth_time_{0.0};
  std::atomic<double> last_mag_time_{0.0};
  std::atomic<double> last_ahrs_time_{0.0};
  std::atomic<double> last_real_dvl_time_{0.0};
  std::atomic<bool> processing_overflow_{false};
  std::atomic<double> last_depth_trigger_time_{0.0};
  std::atomic<double> last_opt_duration_{0.0};
  std::map<rclcpp::Time, gtsam::Key> time_to_key_;

  // --- GTSAM Objects ---
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> inc_smoother_;
  std::unique_ptr<gtsam::ISAM2> isam_;

  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrator_;
  std::unique_ptr<utils::DVLPreintegrator> dvl_preintegrator_;

  gtsam::Pose3 prev_pose_;
  gtsam::Vector3 prev_vel_;
  gtsam::imuBias::ConstantBias prev_imu_bias_;

  gtsam::Vector3 last_dvl_velocity_ = gtsam::Vector3::Zero();
  gtsam::Matrix3 last_dvl_covariance_ = gtsam::Matrix3::Zero();

  // --- Initial Averaged Measurements ---
  sensor_msgs::msg::Imu::SharedPtr initial_imu_;
  nav_msgs::msg::Odometry::SharedPtr initial_gps_;
  nav_msgs::msg::Odometry::SharedPtr initial_depth_;
  sensor_msgs::msg::Imu::SharedPtr initial_ahrs_;
  sensor_msgs::msg::MagneticField::SharedPtr initial_mag_;
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr initial_dvl_;

  // --- Message Queues ---
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_queue_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> depth_queue_;
  std::deque<sensor_msgs::msg::MagneticField::SharedPtr> mag_queue_;
  std::deque<sensor_msgs::msg::Imu::SharedPtr> ahrs_queue_;
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> dvl_queue_;

  // --- Multithreading ---
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;

  std::mutex initialization_mutex_;
  std::mutex optimization_mutex_;
  std::mutex imu_queue_mutex_;
  std::mutex gps_queue_mutex_;
  std::mutex depth_queue_mutex_;
  std::mutex mag_queue_mutex_;
  std::mutex ahrs_queue_mutex_;
  std::mutex dvl_queue_mutex_;

  // --- Transformations ---
  std::string dvl_frame_;
  std::string imu_frame_;

  bool have_dvl_to_base_tf_ = false;
  bool have_imu_to_dvl_tf_ = false;
  bool have_gps_to_dvl_tf_ = false;
  bool have_depth_to_dvl_tf_ = false;
  bool have_mag_to_dvl_tf_ = false;
  bool have_ahrs_to_dvl_tf_ = false;

  geometry_msgs::msg::TransformStamped dvl_to_base_tf_;
  geometry_msgs::msg::TransformStamped imu_to_dvl_tf_;
  geometry_msgs::msg::TransformStamped gps_to_dvl_tf_;
  geometry_msgs::msg::TransformStamped depth_to_dvl_tf_;
  geometry_msgs::msg::TransformStamped mag_to_dvl_tf_;
  geometry_msgs::msg::TransformStamped ahrs_to_dvl_tf_;

  // --- ROS Interfaces ---
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr imu_bias_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr depth_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ahrs_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<factor_graph_node::ParamListener> param_listener_;
  factor_graph_node::Params params_;
};

}  // namespace coug_fgo
