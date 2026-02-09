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
 * @file coug_waypoint_manager.hpp
 * @brief MapViz plugin helper, manages waypoint storage and file I/O.
 * @author Nelson Durrant
 * @date Feb 2026
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

namespace coug_mapviz
{

/**
 * @class CougWaypointManager
 * @brief Handles storage, retrieval, and serialization of waypoints.
 */
class CougWaypointManager
{
public:
  CougWaypointManager() = default;
  ~CougWaypointManager() = default;

  /**
   * @brief Adds a waypoint to the specified topic.
   * @param topic The topic/agent name.
   * @param pose The waypoint pose (in target/local frame).
   */
  void addWaypoint(
    const std::string & topic,
    const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Replaces the waypoint list for a specific topic.
   * @param topic The topic/agent name.
   * @param waypoints The new list of waypoints.
   */
  void setWaypoints(
    const std::string & topic,
    const std::vector<geometry_msgs::msg::Pose> & waypoints);

  /**
   * @brief Gets the waypoints for a specific topic.
   * @param topic The topic name.
   * @return A vector of waypoints.
   */
  std::vector<geometry_msgs::msg::Pose> getWaypoints(
    const std::string & topic) const;

  /**
   * @brief Gets all managed waypoints as a map.
   * @return Map of topic to waypoint list.
   */
  const std::map<std::string, std::vector<geometry_msgs::msg::Pose>> &
  getAllWaypoints() const;

  /**
   * @brief Clears waypoints for a specific topic.
   * @param topic The topic name.
   */
  void clearWaypoints(const std::string & topic);

  /**
   * @brief Clears all waypoints for all topics.
   */
  void clearAllWaypoints();

  /**
   * @brief Saves all waypoints to a JSON file.
   * @param filename The full path to the file.
   * @param topic Optional: Only save this topic.
   * @return True if successful.
   */
  bool saveToFile(
    const std::string & filename,
    const std::string & topic = "") const;

  /**
   * @brief Loads waypoints from a JSON file.
   * @param filename The full path to the file.
   * @param topic Optional: Only load this topic.
   * @return True if successful.
   */
  bool loadFromFile(
    const std::string & filename,
    const std::string & topic = "");

private:
  std::map<std::string, std::vector<geometry_msgs::msg::Pose>> waypoint_map_;
};

}  // namespace coug_mapviz
