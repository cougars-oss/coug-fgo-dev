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
 * @file coug_waypoints_plugin.hpp
 * @brief MapViz plugin for multi-agent waypoint mission planning.
 * @author Nelson Durrant
 * @date Feb 2026
 */

#pragma once

#include <ui_coug_waypoints_config.h>

#include <swri_transform_util/transform.h>

#include <QGLWidget>
#include <QMouseEvent>
#include <QObject>
#include <QPainter>
#include <QTimer>
#include <QWidget>

#include <mapviz/map_canvas.h>
#include <mapviz/mapviz_plugin.h>

#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <coug_mapviz/coug_waypoint_manager.hpp>


namespace coug_mapviz
{

/**
 * @class CougWaypointsPlugin
 * @brief A MapViz plugin that allows users to place, edit, and publish waypoints for multiple agents.
 */
class CougWaypointsPlugin : public mapviz::MapvizPlugin
{
  Q_OBJECT

public:
  // --- Lifecycle & Mapviz Interface ---
  CougWaypointsPlugin();
  ~CougWaypointsPlugin() override;

  /**
   * @brief Initializes the plugin with the map canvas.
   * @param canvas The OpenGL widget to draw on.
   * @return True if initialization succeeds.
   */
  bool Initialize(QGLWidget * canvas) override;

  /**
   * @brief Shuts down the plugin and releases resources.
   */
  void Shutdown() override {}

  /**
   * @brief Draws OpenGL content in the map frame.
   * @param x Camera X position.
   * @param y Camera Y position.
   * @param scale Map scale factor.
   */
  void Draw(double x, double y, double scale) override;

  /**
   * @brief Paints 2D overlays (text, icons) using QPainter.
   * @param painter The QPainter instance.
   * @param x Camera X position.
   * @param y Camera Y position.
   * @param scale Map scale factor.
   */
  void Paint(QPainter * painter, double x, double y, double scale) override;

  /**
   * @brief Handles coordinate transformations (unused).
   */
  void Transform() override {}

  // --- Configuration ---
  /**
   * @brief Loads plugin configuration from YAML.
   * @param node The YAML node containing configuration.
   * @param path Path to the configuration file.
   */
  void LoadConfig(const YAML::Node & node, const std::string & path) override;

  /**
   * @brief Saves plugin configuration to YAML.
   * @param emitter The YAML emitter.
   * @param path Path to the configuration file.
   */
  void SaveConfig(YAML::Emitter & emitter, const std::string & path) override;

  /**
   * @brief Creates and returns the configuration widget.
   * @param parent Parent widget.
   * @return Pointer to the configuration widget.
   */
  QWidget * GetConfigWidget(QWidget * parent) override;

  /**
   * @brief Indicates if the plugin uses QPainter.
   * @return Always true.
   */
  bool SupportsPainting() override {return true;}

protected:
  // --- Error Handling ---
  /**
   * @brief Displays an error message in the status widget.
   * @param message The error message.
   */
  void PrintError(const std::string & message) override;

  /**
   * @brief Displays an info message in the status widget.
   * @param message The info message.
   */
  void PrintInfo(const std::string & message) override;

  /**
   * @brief Displays a warning message in the status widget.
   * @param message The warning message.
   */
  void PrintWarning(const std::string & message) override;

  // --- Event Handling ---
  /**
   * @brief Filters Qt events for mouse interaction.
   * @param object The object receiving the event.
   * @param event The event being processed.
   * @return True if the event was handled.
   */
  bool eventFilter(QObject * object, QEvent * event) override;

  /**
   * @brief Handles mouse press events for adding/selecting waypoints.
   * @param event The mouse event.
   * @return True if handled.
   */
  bool handleMousePress(QMouseEvent * event);

  /**
   * @brief Handles mouse release events for placing/modifying waypoints.
   * @param event The mouse event.
   * @return True if handled.
   */
  bool handleMouseRelease(QMouseEvent * event);

  /**
   * @brief Handles mouse move events for dragging waypoints.
   * @param event The mouse event.
   * @return True if handled.
   */
  bool handleMouseMove(QMouseEvent * event);

protected Q_SLOTS:
  // --- UI Slots ---
  /**
   * @brief Publishes waypoints for the current topic.
   */
  void PublishWaypoints();

  /**
   * @brief Stops the current topic (publishes empty path).
   */
  void Stop();

  /**
   * @brief Clears waypoints for the current topic.
   */
  void Clear();

  /**
   * @brief Opens a dialog to save waypoints to a file.
   */
  void SaveWaypoints();

  /**
   * @brief Opens a dialog to load waypoints from a file.
   */
  void LoadWaypoints();

  /**
   * @brief Handles topic selection changes.
   * @param text The new topic name.
   */
  void TopicChanged(const QString & text);

  /**
   * @brief Discovers available PoseArray topics.
   */
  void DiscoverTopics();

  /**
   * @brief Publishes waypoints for all topics.
   */
  void PublishAll();

  /**
   * @brief Stops all topics.
   */
  void StopAll();

  /**
   * @brief Toggles plugin visibility.
   * @param visible True if visible.
   */
  void VisibilityChanged(bool visible);

  /**
   * @brief Updates the depth of the selected waypoint.
   * @param value New depth value.
   */
  void DepthChanged(double value);

private:
  // --- Components ---
  Ui::coug_waypoints_config ui_;
  QWidget * config_widget_;
  mapviz::MapCanvas * map_canvas_;

  // --- ROS Interface ---
  // Publishers for each topic.
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> publishers_;
  CougWaypointManager manager_;
  std::string current_topic_;

  // --- Interaction State ---
  int selected_point_;
  int dragged_point_;
  bool is_mouse_down_;
  QPointF mouse_down_pos_;
  qint64 mouse_down_time_;

  QTimer * discovery_timer_;

  // --- Helpers ---
  /**
   * @brief Publishes the waypoint list for a specific topic.
   * @param topic The topic to publish to.
   * @param wps The waypoints to publish.
   */
  void PublishTopic(
    const std::string & topic,
    const std::vector<geometry_msgs::msg::Pose> & wps);

  /**
   * @brief Checks if a topic is currently available in the topic selector.
   * @param topic The topic name to check.
   * @return True if the topic is available.
   */
  bool IsTopicAvailable(const std::string & topic);

  /**
   * @brief Finds the closest waypoint to a point on the canvas.
   * @param point The point to check against (in GL coordinates).
   * @param distance Output parameter for the distance found.
   * @return Index of the closest waypoint, or -1 if none found.
   */
  int GetClosestPoint(const QPointF & point, double & distance);

  /**
   * @brief Draws the path lines and points for a topic (GL interface).
   * @param wps The waypoints to draw.
   * @param color The color to draw with.
   * @param transform The current transform to map frame.
   * @param selected_index Index of the selected point (-1 for none).
   */
  void DrawPath(
    const std::vector<geometry_msgs::msg::Pose> & wps,
    const QColor & color,
    const swri_transform_util::Transform & transform,
    int selected_index = -1);

  /**
   * @brief Paints the numeric labels and depth text for a topic (QPainter interface).
   * @param painter The QPainter instance.
   * @param wps The waypoints to label.
   * @param transform The current transform.
   * @param color The text color.
   */
  void PaintLabels(
    QPainter * painter,
    const std::vector<geometry_msgs::msg::Pose> & wps,
    const swri_transform_util::Transform & transform,
    const QColor & color);

  /**
   * @brief Paints the path lines and points for a topic (QPainter interface).
   * @param painter The QPainter instance.
   * @param wps The waypoints to paint.
   * @param color The color to paint with.
   * @param transform The current transform.
   * @param selected_index Index of the selected point (-1 for none).
   */
  void PaintPath(
    QPainter * painter,
    const std::vector<geometry_msgs::msg::Pose> & wps,
    const QColor & color,
    const swri_transform_util::Transform & transform,
    int selected_index = -1);

  /**
   * @brief Transforms local waypoints to the target frame for publishing.
   * @param in Input waypoints (local frame).
   * @param out Output waypoints (target frame).
   * @param transform The transform to apply.
   */
  void TransformWaypoints(
    const std::vector<geometry_msgs::msg::Pose> & in,
    std::vector<geometry_msgs::msg::Pose> & out,
    const swri_transform_util::Transform & transform);
};

}  // namespace coug_mapviz
