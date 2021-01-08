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

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "rclcpp/rclcpp.hpp"
#include "grasp_planning/msg/grasp_pose.hpp"

namespace grasp_execution
{

class FakeGraspPosePublisher : public rclcpp::Node
{
public:
  FakeGraspPosePublisher()
  : Node("fake_grasp_pose_publisher")
  {
    using namespace std::chrono_literals;
    RCLCPP_INFO(this->get_logger(), "Starting...");

    request_.num_objects = 1;
    geometry_msgs::msg::PoseStamped grasp_pose;
    grasp_pose.header.stamp = this->now();
    grasp_pose.header.frame_id = "camera_frame";
    grasp_pose.pose.position.x = 0.012613131664693356;
    grasp_pose.pose.position.y = 0.032275762408971786;
    grasp_pose.pose.position.z = 0.5350000262260437 - 0.023000001907348633 / 2;

    grasp_pose.pose.orientation.x = 0;
    grasp_pose.pose.orientation.y = 0;
    grasp_pose.pose.orientation.z = 0.9996875156442941;
    grasp_pose.pose.orientation.w = 0.024997421165774875;

    // tf2::Quaternion qt;
    // qt.setRPY(0, M_PI, 0);
    // grasp_pose.pose.orientation = tf2::toMsg(qt);

    request_.grasp_poses.push_back(grasp_pose);

    auto object_pose = grasp_pose;
    object_pose.pose.position.x = 0.012877912260591984;
    object_pose.pose.position.y = 0.032953307032585144;
    object_pose.pose.position.z = 0.5350000262260437;

    object_pose.pose.orientation.x = 0;
    object_pose.pose.orientation.y = 0;
    object_pose.pose.orientation.z = 0.9996875156442941;
    object_pose.pose.orientation.w = 0.024997421165774875;

    // qt.setRPY(0, 0, 0);
    // object_pose.pose.orientation = tf2::toMsg(qt);
    request_.object_poses.push_back(object_pose);

    shape_msgs::msg::SolidPrimitive object_shape;
    using shapes = shape_msgs::msg::SolidPrimitive;
    object_shape.type = shapes::BOX;
    object_shape.dimensions.resize(3);
    object_shape.dimensions[shapes::BOX_X] = 0.08691731840372086;
    object_shape.dimensions[shapes::BOX_Y] = 0.16672426462173462;
    object_shape.dimensions[shapes::BOX_Z] = 0.023000001907348633;

    request_.object_shapes.push_back(object_shape);


    publisher_ =
      this->create_publisher<grasp_planning::msg::GraspPose>("grasp_poses", 10);

    rclcpp::sleep_for(
      std::chrono::milliseconds(
        static_cast<int>(2.0 * 1000)));

    RCLCPP_INFO(this->get_logger(), "Starting...");

    publisher_->publish(request_);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

private:
  rclcpp::Publisher<grasp_planning::msg::GraspPose>::SharedPtr publisher_;

  grasp_planning::msg::GraspPose request_;

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
