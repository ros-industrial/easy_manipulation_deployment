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

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "rclcpp/rclcpp.hpp"
#include "emd_msgs/msg/grasp_task.hpp"

#include "grasp_execution/utils.hpp"

namespace grasp_execution
{

class FakeGraspPosePublisher : public rclcpp::Node
{
public:
  FakeGraspPosePublisher()
  : Node("fake_grasp_pose_publisher")
  {
    geometry_msgs::msg::PoseStamped grasp_pose, object_pose;
    grasp_pose.header.stamp = this->now();

    declare_parameter("frame_id");
    declare_parameter("grasp_pose");
    declare_parameter("object_pose");
    declare_parameter("object_dimensions");
    declare_parameter("delay");

    std::string frame_id;

    std::vector<double> grasp_pose_vector{-0.1, 0.4, 0.07, M_PI, 0, 0};
    std::vector<double> object_pose_vector{-0.1, 0.4, 0.05, 0, 0, 0};
    std::vector<double> object_dimensions{0.02, 0.02, 0.1};

    double delay;

    get_parameter_or<std::string>("frame_id", frame_id, "base_link");
    get_parameter_or<std::vector<double>>(
      "grasp_pose", grasp_pose_vector, grasp_pose_vector);

    get_parameter_or<std::vector<double>>(
      "object_pose", object_pose_vector, object_pose_vector);

    get_parameter_or<std::vector<double>>(
      "object_dimensions", object_dimensions, object_dimensions);

    get_parameter_or<double>(
      "delay", delay, 2.0);

    if (!parse_pose_vector(grasp_pose_vector, grasp_pose.pose)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Grasp pose should have 6 or 7 arguments, instead %u is found.",
        grasp_pose_vector.size());
      return;
    }


    if (!parse_pose_vector(object_pose_vector, object_pose.pose)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Object pose should have 6 or 7 arguments, instead %u is found.",
        object_pose_vector.size());
      return;
    }

    if (object_dimensions.size() != 3) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Object dimension shoudl have 3 arguments, instead %u is found.",
        object_dimensions.size());
      return;
    }
    shape_msgs::msg::SolidPrimitive object_shape;
    using shapes = shape_msgs::msg::SolidPrimitive;
    object_shape.type = shapes::BOX;
    object_shape.dimensions.resize(object_dimensions.size());
    for (size_t i = 0; i < object_dimensions.size(); i++) {
      object_shape.dimensions[i] = object_dimensions[i];
    }

    rclcpp::sleep_for(
      std::chrono::milliseconds(static_cast<int>(delay * 1000)));


    grasp_pose.header.frame_id = object_pose.header.frame_id = frame_id;

    grasp_pose.header.stamp = object_pose.header.stamp = this->now();

    request_.task_id = gen_uuid();

    request_.grasp_targets.resize(1);
    request_.grasp_targets[0].target_shape = object_shape;
    request_.grasp_targets[0].target_pose = object_pose;

    request_.grasp_targets[0].grasp_methods.resize(1);

    auto & grasp_method = request_.grasp_targets[0].grasp_methods[0];

    grasp_method.ee_id = "robotiq_2f_gripper";
    grasp_method.grasp_poses.push_back(grasp_pose);
    grasp_method.grasp_ranks = {1.0};

    publisher_ =
      this->create_publisher<emd_msgs::msg::GraspTask>("grasp_request", 10);

    rclcpp::sleep_for(
      std::chrono::milliseconds(
        static_cast<int>(2.0 * 1000)));

    RCLCPP_INFO(this->get_logger(), "Sending fake grasp pose");

    print_pose(grasp_pose);
    print_pose(object_pose);

    publisher_->publish(request_);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

private:
  rclcpp::Publisher<emd_msgs::msg::GraspTask>::SharedPtr publisher_;

  emd_msgs::msg::GraspTask request_;

  float delay_;
};

}  // namespace grasp_execution

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::make_shared<grasp_execution::FakeGraspPosePublisher>();
  rclcpp::shutdown();
  return 0;
}
