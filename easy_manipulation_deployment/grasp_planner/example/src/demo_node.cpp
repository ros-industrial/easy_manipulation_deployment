// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
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

#include "rclcpp/rclcpp.hpp"
#include "emd/grasp_planner/grasp_scene.hpp"

static const rclcpp::Logger & LOGGER_DEMO = rclcpp::get_logger("DemoNode");
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr node =
    rclcpp::Node::make_shared("grasp_planner_demo_node", "", node_options);
  rclcpp::executors::MultiThreadedExecutor executor;
  #if EPD_ENABLED == 1
  if (node->get_parameter("easy_perception_deployment.epd_enabled").as_bool()) {
    RCLCPP_INFO(LOGGER_DEMO, "EPD Workflow Enabled");
    if (node->get_parameter("easy_perception_deployment.tracking_enabled").as_bool()) {
      RCLCPP_INFO(LOGGER_DEMO, "EPD Tracking Enabled");
      grasp_planner::GraspScene<epd_msgs::msg::EPDObjectTracking> demo(node);
      demo.setup(node->get_parameter("easy_perception_deployment.epd_tracking_topic").as_string());
      executor.add_node(demo.node);
      executor.spin();
    } else {
      RCLCPP_INFO(LOGGER_DEMO, "EPD Localization Enabled");
      grasp_planner::GraspScene<epd_msgs::msg::EPDObjectLocalization> demo(node);
      demo.setup(
        node->get_parameter(
          "easy_perception_deployment.epd_localization_topic").as_string());
      executor.add_node(demo.node);
      executor.spin();
    }
  } else {
    RCLCPP_INFO(LOGGER_DEMO, "Direct Workflow Enabled");
    grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2> demo(node);
    demo.setup(node->get_parameter("camera_parameters.point_cloud_topic").as_string());
    executor.add_node(demo.node);
    executor.spin();
  }
  #else
  RCLCPP_INFO(LOGGER_DEMO, "epd_msgs not found, Direct Workflow Enabled");
  grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2> demo(node);
  demo.setup(node->get_parameter("camera_parameters.point_cloud_topic").as_string());
  executor.add_node(demo.node);
  executor.spin();
  #endif

  rclcpp::shutdown();
  std::cout << "Shutting Down" << std::endl;
  return 0;
}
