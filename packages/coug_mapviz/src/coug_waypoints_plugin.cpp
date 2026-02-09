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
 * @file coug_waypoints_plugin.cpp
 * @brief Implementation of the CougWaypointsPlugin.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#include <coug_mapviz/coug_waypoints_plugin.hpp>

#include <swri_transform_util/frames.h>

#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QMouseEvent>
#include <QPainter>

#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(coug_mapviz::CougWaypointsPlugin, mapviz::MapvizPlugin)

namespace coug_mapviz
{

CougWaypointsPlugin::CougWaypointsPlugin()
: MapvizPlugin(),
  ui_(),
  config_widget_(new QWidget()),
  map_canvas_(nullptr),
  selected_point_(-1),
  dragged_point_(-1),
  is_mouse_down_(false),
  mouse_down_time_(0)
{
  ui_.setupUi(config_widget_);

  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(p);
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::darkGreen);
  ui_.status->setPalette(p3);

  discovery_timer_ = new QTimer(this);
  QObject::connect(
    discovery_timer_, SIGNAL(timeout()), this,
    SLOT(DiscoverTopics()));

  QObject::connect(
    ui_.topic_selector,
    SIGNAL(currentTextChanged(const QString&)), this,
    SLOT(TopicChanged(const QString&)));
  QObject::connect(
    ui_.publish, SIGNAL(clicked()), this,
    SLOT(PublishWaypoints()));
  QObject::connect(ui_.stop, SIGNAL(clicked()), this, SLOT(Stop()));
  QObject::connect(ui_.clear, SIGNAL(clicked()), this, SLOT(Clear()));

  QObject::connect(ui_.save, SIGNAL(clicked()), this, SLOT(SaveWaypoints()));
  QObject::connect(ui_.load, SIGNAL(clicked()), this, SLOT(LoadWaypoints()));

  QObject::connect(
    this, SIGNAL(VisibleChanged(bool)), this,
    SLOT(VisibilityChanged(bool)));
  QObject::connect(
    ui_.depth_editor, SIGNAL(valueChanged(double)), this,
    SLOT(DepthChanged(double)));
}

CougWaypointsPlugin::~CougWaypointsPlugin()
{
  if (map_canvas_) {
    map_canvas_->removeEventFilter(this);
  }
}

bool CougWaypointsPlugin::Initialize(QGLWidget * canvas)
{
  map_canvas_ = dynamic_cast<mapviz::MapCanvas *>(canvas);
  map_canvas_->installEventFilter(this);

  discovery_timer_->start(1000);
  DiscoverTopics();

  initialized_ = true;
  return true;
}

void CougWaypointsPlugin::VisibilityChanged(bool visible)
{
  if (visible) {
    map_canvas_->installEventFilter(this);
  } else {
    map_canvas_->removeEventFilter(this);
  }
}

void CougWaypointsPlugin::DiscoverTopics()
{
  auto topics_and_types = node_->get_topic_names_and_types();
  for (const auto &[topic, types] : topics_and_types) {
    for (const auto & type : types) {
      if (type == "geometry_msgs/msg/PoseArray") {
        if (ui_.topic_selector->findText(QString::fromStdString(topic)) == -1) {
          ui_.topic_selector->addItem(QString::fromStdString(topic));
        }
        break;
      }
    }
  }
}

void CougWaypointsPlugin::TopicChanged(const QString & text)
{
  current_topic_ = text.toStdString();
  selected_point_ = -1;
  ui_.depth_editor->setEnabled(false);

  auto wps = manager_.getWaypoints(current_topic_);
  if (wps.empty()) {
    PrintInfo("Click to add waypoints");
  } else {
    PrintInfo(
      current_topic_ + " (" + std::to_string(wps.size()) +
      " waypoints)");
  }
}

void CougWaypointsPlugin::PublishWaypoints()
{
  if (ui_.apply_all->isChecked()) {
    PublishAll();
  } else if (!current_topic_.empty()) {
    auto wps = manager_.getWaypoints(current_topic_);
    PublishTopic(current_topic_, wps);

    if (wps.empty()) {
      PrintWarning("Stopped");
    } else {
      PrintInfo("Published " + std::to_string(wps.size()) + " waypoints");
    }
  } else {
    PrintError("No topic selected");
  }
}

void CougWaypointsPlugin::Stop()
{
  if (ui_.apply_all->isChecked()) {
    StopAll();
  } else if (!current_topic_.empty()) {
    PublishTopic(current_topic_, {});
    PrintWarning("Stopped");
  }
}

void CougWaypointsPlugin::PublishAll()
{
  int count = 0;
  for (const auto &[topic, wps] : manager_.getAllWaypoints()) {
    PublishTopic(topic, wps);
    count++;
  }
  PrintInfo("Published all (" + std::to_string(count) + " topics)");
}

void CougWaypointsPlugin::StopAll()
{
  for (const auto &[topic, wps] : manager_.getAllWaypoints()) {
    PublishTopic(topic, {});
  }
  PrintWarning(
    "Stopped all (" +
    std::to_string(manager_.getAllWaypoints().size()) + " topics)");
}

void CougWaypointsPlugin::PublishTopic(
  const std::string & topic,
  const std::vector<geometry_msgs::msg::Pose> & wps)
{
  if (publishers_.find(topic) == publishers_.end()) {
    publishers_[topic] = node_->create_publisher<geometry_msgs::msg::PoseArray>(
      topic, rclcpp::QoS(1));
  }

  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(
      target_frame_,
      swri_transform_util::_wgs84_frame,
      transform))
  {
    PrintError("No transform for " + topic);
    return;
  }

  std::vector<geometry_msgs::msg::Pose> transformed_wps;
  TransformWaypoints(wps, transformed_wps, transform);

  auto pose_array = std::make_unique<geometry_msgs::msg::PoseArray>();
  pose_array->header.frame_id = target_frame_;
  pose_array->header.stamp = node_->now();
  pose_array->poses = transformed_wps;

  publishers_[topic]->publish(*pose_array);
}

bool CougWaypointsPlugin::IsTopicAvailable(const std::string & topic)
{
  return ui_.topic_selector->findText(QString::fromStdString(topic)) != -1;
}

void CougWaypointsPlugin::Clear()
{
  if (ui_.apply_all->isChecked()) {
    manager_.clearAllWaypoints();
  } else if (!current_topic_.empty()) {
    manager_.clearWaypoints(current_topic_);
  }

  ui_.depth_editor->setValue(0.0);
  selected_point_ = -1;
  dragged_point_ = -1;
  ui_.depth_editor->setEnabled(false);
  PrintInfo("Cleared waypoints");
}

void CougWaypointsPlugin::SaveWaypoints()
{
  QString path = QDir::homePath() + "/coug_ws/src/coug_mapviz/waypoints";
  QDir dir(path);
  if (!dir.exists()) {
    dir.mkpath(".");
  }

  QString filename = QFileDialog::getSaveFileName(
    config_widget_, "Save Mission", path, "JSON Files (*.json)");
  if (filename.isEmpty()) {return;}
  if (!filename.endsWith(".json", Qt::CaseInsensitive)) {filename += ".json";}

  std::string topic_to_save = "";
  if (!ui_.apply_all->isChecked()) {
    if (current_topic_.empty()) {
      PrintError("No topic selected to save");
      return;
    }
    topic_to_save = current_topic_;
  }

  if (manager_.saveToFile(filename.toStdString(), topic_to_save)) {
    if (topic_to_save.empty()) {
      PrintInfo(
        "Saved all (" +
        std::to_string(manager_.getAllWaypoints().size()) + " topics)");
    } else {
      PrintInfo("Saved topic: " + topic_to_save);
    }
  } else {
    PrintError("Failed to save file");
  }
}

void CougWaypointsPlugin::LoadWaypoints()
{
  QString path = QDir::homePath() + "/coug_ws/src/coug_mapviz/waypoints";
  QString filename = QFileDialog::getOpenFileName(
    config_widget_, "Load Mission", path, "JSON Files (*.json)");
  if (filename.isEmpty()) {return;}

  std::string topic_to_load = "";
  if (!ui_.apply_all->isChecked()) {
    if (current_topic_.empty()) {
      PrintError("No topic selected to load");
      return;
    }
    topic_to_load = current_topic_;
  }

  if (manager_.loadFromFile(filename.toStdString(), topic_to_load)) {
    TopicChanged(QString::fromStdString(current_topic_));
    if (topic_to_load.empty()) {
      PrintInfo(
        "Loaded all topics (" +
        std::to_string(manager_.getAllWaypoints().size()) + " total)");
    } else {
      auto wps = manager_.getWaypoints(topic_to_load);
      PrintInfo(
        "Loaded topic: " + topic_to_load + " (" +
        std::to_string(wps.size()) + " waypoints)");
    }
  } else {
    PrintError("Failed to load file");
  }
}

void CougWaypointsPlugin::Draw(double x, double y, double scale)
{
  (void)x;
  (void)y;
  (void)scale;

  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(
      target_frame_,
      swri_transform_util::_wgs84_frame,
      transform))
  {
    return;
  }

  glLineWidth(2);

  for (const auto &[topic, wps] : manager_.getAllWaypoints()) {
    if (topic != current_topic_ && IsTopicAvailable(topic)) {
      DrawPath(wps, QColor(Qt::white), transform, -1);
    }
  }
}

void CougWaypointsPlugin::DrawPath(
  const std::vector<geometry_msgs::msg::Pose> & wps, const QColor & color,
  const swri_transform_util::Transform & transform, int selected_index)
{
  if (color == Qt::blue) {
    glColor4f(0.0, 0.0, 1.0, 1.0);
  } else {
    glColor4f(1.0, 1.0, 1.0, 1.0);
  }

  glBegin(GL_LINE_STRIP);
  for (const auto & wp : wps) {
    tf2::Vector3 point(wp.position.x, wp.position.y, 0);
    point = transform * point;
    glVertex2d(point.x(), point.y());
  }
  glEnd();

  glPointSize(20);
  glBegin(GL_POINTS);
  for (size_t i = 0; i < wps.size(); ++i) {
    if (static_cast<int>(i) == selected_index) {
      glColor4f(1.0, 1.0, 0.0, 1.0);
    } else if (color == Qt::blue) {
      glColor4f(0.0, 1.0, 1.0, 1.0);
    } else {
      glColor4f(0.5, 0.5, 0.5, 1.0);
    }

    tf2::Vector3 point(wps[i].position.x, wps[i].position.y, 0);
    point = transform * point;
    glVertex2d(point.x(), point.y());
  }
  glEnd();
}

void CougWaypointsPlugin::Paint(
  QPainter * painter, double x, double y,
  double scale)
{
  (void)x;
  (void)y;
  (void)scale;

  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(
      target_frame_,
      swri_transform_util::_wgs84_frame,
      transform))
  {
    return;
  }

  painter->save();
  painter->resetTransform();

  painter->setFont(QFont("DejaVu Sans Mono", 10, QFont::Bold));
  for (const auto &[topic, wps] : manager_.getAllWaypoints()) {
    if (topic != current_topic_ && IsTopicAvailable(topic)) {
      PaintLabels(painter, wps, transform, QColor(255, 255, 255, 200));
    }
  }

  if (!current_topic_.empty()) {
    auto wps = manager_.getWaypoints(current_topic_);
    if (!wps.empty()) {
      PaintPath(painter, wps, QColor(Qt::blue), transform, selected_point_);
    }
  }

  painter->setFont(QFont("DejaVu Sans Mono", 10, QFont::Bold));
  if (!current_topic_.empty()) {
    auto wps = manager_.getWaypoints(current_topic_);
    if (!wps.empty()) {
      PaintLabels(painter, wps, transform, Qt::white);
    }
  }
  painter->restore();
}

void CougWaypointsPlugin::PaintPath(
  QPainter * painter, const std::vector<geometry_msgs::msg::Pose> & wps,
  const QColor & color, const swri_transform_util::Transform & transform,
  int selected_index)
{
  QVector<QPointF> points;
  for (const auto & wp : wps) {
    tf2::Vector3 point(wp.position.x, wp.position.y, 0);
    point = transform * point;
    points.push_back(
      map_canvas_->FixedFrameToMapGlCoord(QPointF(point.x(), point.y())));
  }

  QPen pen(color, 2);
  painter->setPen(pen);
  painter->drawPolyline(points);
  for (int i = 0; i < points.size(); ++i) {
    if (i == selected_index) {
      painter->setPen(QPen(Qt::yellow, 20, Qt::SolidLine, Qt::RoundCap));
      painter->drawPoint(points[i]);
    } else if (color == Qt::blue) {
      painter->setPen(QPen(Qt::cyan, 20, Qt::SolidLine, Qt::RoundCap));
      painter->drawPoint(points[i]);
    } else {
      painter->setPen(QPen(Qt::gray, 20, Qt::SolidLine, Qt::RoundCap));
      painter->drawPoint(points[i]);
    }
  }
}

void CougWaypointsPlugin::PaintLabels(
  QPainter * painter, const std::vector<geometry_msgs::msg::Pose> & wps,
  const swri_transform_util::Transform & transform, const QColor & color)
{
  for (size_t i = 0; i < wps.size(); i++) {
    tf2::Vector3 point(wps[i].position.x, wps[i].position.y, 0);
    point = transform * point;
    QPointF gl_point =
      map_canvas_->FixedFrameToMapGlCoord(QPointF(point.x(), point.y()));

    painter->setPen(QPen(color));

    QPointF depth_text_corner(gl_point.x() - 50, gl_point.y() + 15);
    QRectF depth_text_rect(depth_text_corner, QSizeF(100, 20));
    QString depth_text = QString::number(wps[i].position.z, 'f', 1) + "m";
    painter->drawText(
      depth_text_rect, Qt::AlignHCenter | Qt::AlignTop,
      depth_text);

    painter->setPen(QPen(color == Qt::white ? Qt::black : color));
    QPointF num_corner(gl_point.x() - 20, gl_point.y() - 20);
    QRectF num_rect(num_corner, QSizeF(40, 40));
    painter->drawText(
      num_rect, Qt::AlignHCenter | Qt::AlignVCenter,
      QString::number(i + 1));
  }
}

void CougWaypointsPlugin::TransformWaypoints(
  const std::vector<geometry_msgs::msg::Pose> & in,
  std::vector<geometry_msgs::msg::Pose> & out,
  const swri_transform_util::Transform & transform)
{
  out = in;
  for (size_t i = 0; i < in.size(); i++) {
    tf2::Vector3 position(in[i].position.x, in[i].position.y, 0.0);
    position = transform * position;
    out[i].position.x = position.x();
    out[i].position.y = position.y();
    out[i].position.z = in[i].position.z;
  }
}

void CougWaypointsPlugin::DepthChanged(double value)
{
  if (current_topic_.empty()) {return;}

  auto wps = manager_.getWaypoints(current_topic_);
  if (selected_point_ >= 0 &&
    static_cast<size_t>(selected_point_) < wps.size())
  {
    wps[selected_point_].position.z = value;
    manager_.setWaypoints(current_topic_, wps);
  }
}

bool CougWaypointsPlugin::eventFilter(QObject * object, QEvent * event)
{
  (void)object;
  switch (event->type()) {
    case QEvent::MouseButtonPress:
      return handleMousePress(dynamic_cast<QMouseEvent *>(event));
    case QEvent::MouseButtonRelease:
      return handleMouseRelease(dynamic_cast<QMouseEvent *>(event));
    case QEvent::MouseMove:
      return handleMouseMove(dynamic_cast<QMouseEvent *>(event));
    default:
      return false;
  }
}

int CougWaypointsPlugin::GetClosestPoint(
  const QPointF & point,
  double & distance)
{
  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(
      target_frame_,
      swri_transform_util::_wgs84_frame,
      transform))
  {
    return -1;
  }

  int closest = -1;
  distance = std::numeric_limits<double>::max();
  auto wps = manager_.getWaypoints(current_topic_);

  for (size_t i = 0; i < wps.size(); i++) {
    tf2::Vector3 waypoint(wps[i].position.x, wps[i].position.y, 0);
    waypoint = transform * waypoint;
    QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(
      QPointF(waypoint.x(), waypoint.y()));

    double d = QLineF(transformed, point).length();
    if (d < distance) {
      distance = d;
      closest = static_cast<int>(i);
    }
  }
  return closest;
}

bool CougWaypointsPlugin::handleMousePress(QMouseEvent * event)
{
  if (current_topic_.empty()) {return false;}

  dragged_point_ = -1;
  double distance = 0.0;
  int closest_point = GetClosestPoint(event->localPos(), distance);

  if (event->button() == Qt::LeftButton) {
    is_mouse_down_ = true;
    mouse_down_pos_ = event->localPos();
    mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();

    if (distance < 15) {
      dragged_point_ = closest_point;
      return true;
    }
  } else if (event->button() == Qt::RightButton) {
    if (distance < 15) {
      auto wps = manager_.getWaypoints(current_topic_);
      wps.erase(wps.begin() + closest_point);
      manager_.setWaypoints(current_topic_, wps);

      if (selected_point_ == closest_point) {
        selected_point_ = -1;
        ui_.depth_editor->setEnabled(false);
      }
      return true;
    }
  }
  return false;
}

bool CougWaypointsPlugin::handleMouseRelease(QMouseEvent * event)
{
  if (current_topic_.empty()) {return false;}

  qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
  qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

  if (dragged_point_ != -1) {
    if (distance <= 5.0 && msecsDiff < 500) {
      selected_point_ = dragged_point_;
      ui_.depth_editor->setEnabled(true);
      ui_.depth_editor->blockSignals(true);
      auto wps = manager_.getWaypoints(current_topic_);
      ui_.depth_editor->setValue(wps[selected_point_].position.z);
      ui_.depth_editor->blockSignals(false);
    }
    dragged_point_ = -1;
    return true;
  }

  if (is_mouse_down_) {
    if (msecsDiff < 500 && distance <= 5.0) {
      QPointF transformed =
        map_canvas_->MapGlCoordToFixedFrame(event->localPos());
      swri_transform_util::Transform transform;
      if (tf_manager_->GetTransform(
          swri_transform_util::_wgs84_frame,
          target_frame_, transform))
      {
        tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;

        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = ui_.depth_editor->value();

        manager_.addWaypoint(current_topic_, pose);
      }
    }
  }

  is_mouse_down_ = false;
  dragged_point_ = -1;
  return false;
}

bool CougWaypointsPlugin::handleMouseMove(QMouseEvent * event)
{
  if (current_topic_.empty()) {return false;}

  if (dragged_point_ >= 0) {
    if (selected_point_ != -1) {
      selected_point_ = -1;
      ui_.depth_editor->setEnabled(false);
    }

    swri_transform_util::Transform transform;
    if (tf_manager_->GetTransform(
        swri_transform_util::_wgs84_frame,
        target_frame_, transform))
    {
      QPointF transformed =
        map_canvas_->MapGlCoordToFixedFrame(event->localPos());
      tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
      position = transform * position;

      auto wps = manager_.getWaypoints(current_topic_);
      wps[dragged_point_].position.x = position.x();
      wps[dragged_point_].position.y = position.y();
      manager_.setWaypoints(current_topic_, wps);
    }
    return true;
  }
  return false;
}

void CougWaypointsPlugin::LoadConfig(
  const YAML::Node & node,
  const std::string & path)
{
  (void)node;
  (void)path;
}

void CougWaypointsPlugin::SaveConfig(
  YAML::Emitter & emitter,
  const std::string & path)
{
  (void)emitter;
  (void)path;
}

QWidget * CougWaypointsPlugin::GetConfigWidget(QWidget * parent)
{
  config_widget_->setParent(parent);
  return config_widget_;
}

void CougWaypointsPlugin::PrintError(const std::string & message)
{
  PrintErrorHelper(ui_.status, message, 1.0);
}

void CougWaypointsPlugin::PrintInfo(const std::string & message)
{
  PrintInfoHelper(ui_.status, message, 1.0);
}

void CougWaypointsPlugin::PrintWarning(const std::string & message)
{
  PrintWarningHelper(ui_.status, message, 1.0);
}

}  // namespace coug_mapviz
