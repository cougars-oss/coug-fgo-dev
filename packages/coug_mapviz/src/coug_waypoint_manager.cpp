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
 * @file coug_waypoint_manager.cpp
 * @brief Implementation of the CougWaypointManager.
 * @author Nelson Durrant
 * @date Feb 2026
 */

#include <coug_mapviz/coug_waypoint_manager.hpp>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>

#include <map>
#include <string>
#include <vector>

namespace coug_mapviz
{

void CougWaypointManager::addWaypoint(
  const std::string & topic,
  const geometry_msgs::msg::Pose & pose)
{
  waypoint_map_[topic].push_back(pose);
}

void CougWaypointManager::setWaypoints(
  const std::string & topic,
  const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  waypoint_map_[topic] = waypoints;
}

std::vector<geometry_msgs::msg::Pose> CougWaypointManager::getWaypoints(
  const std::string & topic) const
{
  if (waypoint_map_.find(topic) != waypoint_map_.end()) {
    return waypoint_map_.at(topic);
  }
  return {};
}

const std::map<std::string, std::vector<geometry_msgs::msg::Pose>> &
CougWaypointManager::getAllWaypoints() const
{
  return waypoint_map_;
}

void CougWaypointManager::clearWaypoints(const std::string & topic)
{
  waypoint_map_[topic].clear();
}

void CougWaypointManager::clearAllWaypoints() {waypoint_map_.clear();}

bool CougWaypointManager::saveToFile(
  const std::string & filename,
  const std::string & specific_topic) const
{
  QJsonObject multi_topic_obj;

  for (const auto & [topic, wps] : waypoint_map_) {
    if (!specific_topic.empty() && topic != specific_topic) {
      continue;
    }

    QJsonArray waypoints_array;
    for (const auto & wp : wps) {
      QJsonObject wp_obj;

      wp_obj["lon"] = wp.position.x;
      wp_obj["lat"] = wp.position.y;
      wp_obj["z"] = wp.position.z;

      waypoints_array.append(wp_obj);
    }
    multi_topic_obj[QString::fromStdString(topic)] = waypoints_array;
  }

  QJsonDocument doc(multi_topic_obj);
  QFile file(QString::fromStdString(filename));
  if (file.open(QIODevice::WriteOnly)) {
    file.write(doc.toJson());
    file.close();
    return true;
  }
  return false;
}

bool CougWaypointManager::loadFromFile(
  const std::string & filename,
  const std::string & specific_topic)
{
  QFile file(QString::fromStdString(filename));
  if (!file.open(QIODevice::ReadOnly)) {
    return false;
  }

  QByteArray data = file.readAll();
  QJsonDocument doc = QJsonDocument::fromJson(data);

  if (!doc.isObject()) {
    return false;
  }

  QJsonObject obj = doc.object();
  int loaded_count = 0;

  for (const QString & topic_key : obj.keys()) {
    std::string topic_str = topic_key.toStdString();

    if (!specific_topic.empty() && topic_str != specific_topic) {
      continue;
    }

    std::vector<geometry_msgs::msg::Pose> wps;
    QJsonArray array = obj[topic_key].toArray();

    for (const auto & val : array) {
      QJsonObject wp_obj = val.toObject();
      geometry_msgs::msg::Pose pose;

      if (wp_obj.contains("lat") && wp_obj.contains("lon")) {
        pose.position.x = wp_obj["lon"].toDouble();
        pose.position.y = wp_obj["lat"].toDouble();
        pose.position.z = wp_obj["z"].toDouble();
        wps.push_back(pose);
      }
    }
    waypoint_map_[topic_str] = wps;
    loaded_count++;
  }

  return loaded_count > 0;
}

}  // namespace coug_mapviz
