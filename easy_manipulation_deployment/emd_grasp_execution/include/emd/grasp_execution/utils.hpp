// Copyright 2020 ROS Industrial Consortium Asia Pacific
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

#ifndef EMD__GRASP_EXECUTION__UTILS_HPP_
#define EMD__GRASP_EXECUTION__UTILS_HPP_

#include <iostream>
#include <string>
#include <vector>

#include "boost/uuid/uuid.hpp"
#include "boost/uuid/random_generator.hpp"
#include "boost/uuid/uuid_io.hpp"

#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2/impl/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace grasp_execution
{

/// Generate UUID
/// \return UUID in string
inline std::string gen_uuid()
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  return boost::uuids::to_string(uuid);
}

/// Print Pose Message TODO(Briancbn): default std::cout doesn't seems to work here.
inline void print_pose(
  const geometry_msgs::msg::Pose & _pose,
  std::ostream & _out = std::cout,
  bool _euler = true)
{
  _out << "Position:" << std::endl;

  _out << "X: " << _pose.position.x << std::endl;
  _out << "Y: " << _pose.position.y << std::endl;
  _out << "Z: " << _pose.position.z << std::endl << std::endl;

  _out << "Orientation:" << std::endl;
  if (_euler) {
    double yaw, pitch, roll;
    tf2::Quaternion q_tmp{
      _pose.orientation.x,
      _pose.orientation.y,
      _pose.orientation.z,
      _pose.orientation.w};
    tf2::impl::getEulerYPR(q_tmp, yaw, pitch, roll);
    _out << "Yaw: " << yaw << std::endl;
    _out << "Pitch: " << pitch << std::endl;
    _out << "Roll: " << roll << std::endl << std::endl;
  } else {
    _out << "X: " << _pose.orientation.x << std::endl;
    _out << "Y: " << _pose.orientation.y << std::endl;
    _out << "Z: " << _pose.orientation.z << std::endl;
    _out << "W: " << _pose.orientation.w << std::endl << std::endl;
  }
}

/// Print PoseStamped Message
inline void print_pose(
  const geometry_msgs::msg::PoseStamped & _pose,
  std::ostream & _out = std::cout,
  bool _euler = true)
{
  _out << "Frame ID: " << _pose.header.frame_id << std::endl;

  print_pose(_pose.pose, _out, _euler);
}

/// Use ROS to print PoseStamped message
inline void print_pose_ros(
  const rclcpp::Logger logger,
  const geometry_msgs::msg::Pose & _pose,
  bool _euler = true)
{
  std::ostringstream oss;
  print_pose(_pose, oss, _euler);
  RCLCPP_INFO(logger, oss.str().c_str());
}

/// Use ROS to print PoseStamped message
inline void print_pose_ros(
  const rclcpp::Logger logger,
  const geometry_msgs::msg::PoseStamped & _pose,
  bool _euler = true)
{
  std::ostringstream oss;
  oss << std::endl;
  print_pose(_pose, oss, _euler);
  RCLCPP_INFO(logger, oss.str().c_str());
}

/// Transform pose to target end reference frame
inline void to_frame(
  const geometry_msgs::msg::PoseStamped & in_,
  geometry_msgs::msg::PoseStamped & out_,
  const std::string & _target_frame,
  const tf2_ros::Buffer & _buffer)
{
  auto base_frame2target_frame = _buffer.lookupTransform(
    _target_frame, in_.header.frame_id, rclcpp::Time());

  tf2::doTransform(in_, out_, base_frame2target_frame);
}

/// Parse vector with 6 or 7 variables to a pose
inline bool parse_pose_vector(
  const std::vector<double> & param,
  geometry_msgs::msg::Pose & pose)
{
  // Location is x y z qx qy qz qw
  if (param.size() == 7) {
    pose.position.x = param[0];
    pose.position.y = param[1];
    pose.position.z = param[2];
    tf2::Quaternion temp_qr(
      param[3],
      param[4],
      param[5],
      param[6]);
    pose.orientation = tf2::toMsg(temp_qr.normalized());
    return true;
  } else if (param.size() == 6) {
    // Location is x y z roll pitch yaw
    pose.position.x = param[0];
    pose.position.y = param[1];
    pose.position.z = param[2];
    tf2::Quaternion temp_qr;
    temp_qr.setRPY(
      param[3],
      param[4],
      param[5]);
    pose.orientation = tf2::toMsg(temp_qr);
    return true;
  } else {
    // Location invalid
    return false;
  }
}

template<typename T>
/// Get parameter (declare if doesn't allow overload)
/**
 * Referenced from moveit servo.
 */
inline void declare_or_get_param(
  T & output_value,
  const std::string & param_name,
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Logger & logger,
  const T & default_value = T{})
{
  try {
    if (node->has_parameter(param_name)) {
      node->get_parameter_or<T>(param_name, output_value, default_value);
    } else {
      output_value = node->declare_parameter<T>(param_name, default_value);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    // Catch a <double> parameter written in the yaml as "1" being considered an <int>
    if (std::is_same<T, double>::value) {
      node->undeclare_parameter(param_name);
      output_value = node->declare_parameter<int>(param_name, 0);
    } else {
      RCLCPP_ERROR(
        logger,
        "Error getting parameter \'%s\', check parameter type in YAML file.",
        param_name.c_str());
      throw e;
    }
  }
  RCLCPP_INFO_STREAM(
    logger,
    "Found parameter - " << param_name << ": " << output_value);
}

}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__UTILS_HPP_
