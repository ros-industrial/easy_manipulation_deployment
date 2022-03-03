// Copyright 2022 ROS Industrial Consortium Asia Pacific
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

#include "emd/waypoint_execution/waypoint_parser.hpp"

bool emd::WayPointParser::internal::get_json_offset(
  Json::Value json_object,
  emd::WayPoint & waypoint)
{
  Json::Value position_object = json_object.get("position", "default");
  Json::Value orientation_object = json_object.get("orientation", "default");
  waypoint.position.push_back(position_object.get("x", "default").asDouble());
  waypoint.position.push_back(position_object.get("y", "default").asDouble());
  waypoint.position.push_back(position_object.get("z", "default").asDouble());

  waypoint.rpy.push_back(orientation_object.get("r", "default").asDouble());
  waypoint.rpy.push_back(orientation_object.get("p", "default").asDouble());
  waypoint.rpy.push_back(orientation_object.get("y", "default").asDouble());
  return true;
}

bool emd::WayPointParser::internal::add_rpy_to_pose(
  geometry_msgs::msg::PoseStamped & target_pose,
  float roll,
  float pitch,
  float yaw)
{
  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::convert(target_pose.pose.orientation, q_orig);
  q_rot.setRPY(roll, pitch, yaw);
  q_new = q_rot * q_orig;  // Calculate the new orientation
  q_new.normalize();
  tf2::convert(q_new, target_pose.pose.orientation);
  return true;
}

bool emd::WayPointParser::load_waypoints(
  std::vector<emd::WayPoint> & waypoints,
  const std::string waypoint_filepath)
{
  std::filesystem::path path{waypoint_filepath};
  if (!std::filesystem::exists(path)) {
    RCLCPP_ERROR(
      LOGGER_WAYPOINT,
      "Action Json filepath: \"" + waypoint_filepath + " \"not found!");
    return false;
  }
  std::ifstream ifs;
  ifs.open(waypoint_filepath);

  Json::Value root;
  Json::CharReaderBuilder builder;
  builder["collectComments"] = true;
  JSONCPP_STRING errs;
  if (!parseFromStream(builder, ifs, &root, &errs)) {
    RCLCPP_ERROR(LOGGER_WAYPOINT, errs);
  }
  Json::Value action_name_obj = root.get("actions", "default");
  std::vector<std::string> action_names;
  for (auto itr : action_name_obj) {
    action_names.push_back(itr.asString());
  }

  for (std::string action_name : action_names) {
    Json::Value action_object = root.get(action_name, "default");
    std::string action_type = action_object.get("action_type", "default").asString();
    bool is_relative = action_object.get("relative_waypoint", "default").asBool();

    if (action_type.compare("move") == 0) {
      emd::WayPoint waypoint(is_relative, action_type, action_name);
      emd::WayPointParser::internal::get_json_offset(action_object, waypoint);
      waypoints.push_back(waypoint);
    } else if (action_type.compare("collision") == 0) {
      std::vector<int> collision_axis;
      const Json::Value axis_array = action_object["collision_axis"];
      for (unsigned int index = 0; index < axis_array.size(); ++index) {
        collision_axis.push_back(axis_array[index].asInt());
      }
      if (collision_axis.size() == 0) {
        ifs.close();
        return false;
      } else {
        emd::WayPoint waypoint(is_relative, action_type, action_name, collision_axis);
        emd::WayPointParser::internal::get_json_offset(action_object, waypoint);
        waypoints.push_back(waypoint);
      }
    } else if (action_type.compare("grasp") == 0) {
      // float grasp_width = action_object.get("grasp_width", "default").asDouble();
      emd::WayPoint waypoint(action_type, action_name);
      waypoints.push_back(waypoint);
    } else if (action_type.compare("release") == 0) {
      // float grasp_width = action_object.get("grasp_width", "default").asDouble();
      emd::WayPoint waypoint(action_type, action_name);
      waypoints.push_back(waypoint);
    } else {
      RCLCPP_ERROR(
        LOGGER_WAYPOINT,
        "Action Type not recognized, please check your action json file!");
      ifs.close();
      return false;
    }
  }
  ifs.close();
  return true;
}
