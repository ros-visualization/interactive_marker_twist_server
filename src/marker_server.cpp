// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <tf2/utils.h>

namespace interactive_marker_twist_server
{

class TwistServerNode : public rclcpp::Node
{
public:
  TwistServerNode();

  ~TwistServerNode() = default;

private:
  void getParameters();
  void createInteractiveMarkers();
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;

  std::map<std::string, double> linear_drive_scale_map;
  std::map<std::string, double> max_positive_linear_velocity_map;
  std::map<std::string, double> max_negative_linear_velocity_map;

  double angular_drive_scale;
  double max_angular_velocity;
  double marker_size_scale;

  std::string link_name;
  std::string robot_name;
}; // class TwistServerNode

TwistServerNode::TwistServerNode() : rclcpp::Node("twist_server_node")
{
  getParameters();
  vel_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  server = std::make_unique<interactive_markers::InteractiveMarkerServer>("twist_server", get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(), get_node_topics_interface(), get_node_services_interface());
  createInteractiveMarkers();
  RCLCPP_INFO(get_logger(), "[interacive_marker_twist_server] Initialized.");
}

void TwistServerNode::getParameters()
{
  rclcpp::Parameter link_name_param;
  rclcpp::Parameter robot_name_param;
  rclcpp::Parameter linear_scale_param;
  rclcpp::Parameter max_positive_linear_velocity_param;
  rclcpp::Parameter max_negative_linear_velocity_param;

  if (this->get_parameter("link_name", link_name_param))
  {
    link_name = link_name_param.as_string();
  }
  else
  {
    link_name = "base_link";
  }

  if (this->get_parameter("robot_name", robot_name_param))
  {
    robot_name = robot_name_param.as_string();
  }
  else
  {
    robot_name = "robot";
  }

  if (this->get_parameter("linear_scale", linear_scale_param))
  {
    this->get_parameter("max_positive_linear_velocity", max_positive_linear_velocity_param);
    this->get_parameter("max_negative_linear_velocity", max_negative_linear_velocity_param);
    linear_drive_scale_map = linear_scale_param.as_double();
    max_positive_linear_velocity_map = max_positive_linear_velocity_param.as_double();
    max_negative_linear_velocity_map = max_negative_linear_velocity_param.as_double();
  }
  else
  {
    linear_drive_scale_map = 1.0;
    max_positive_linear_velocity_map = 1.0;
    max_negative_linear_velocity_map = -1.0;
  }

  angular_drive_scale = 2.2;
  max_angular_velocity = 2.2;
  marker_size_scale = 1.0;
}

void TwistServerNode::createInteractiveMarkers()
{
  visualization_msgs::msg::InteractiveMarker interactive_marker;
  interactive_marker.header.frame_id = link_name;
  interactive_marker.name = robot_name + "_twist_marker";
  interactive_marker.description = "twist controller for " + robot_name;
  interactive_marker.scale = marker_size_scale;

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;

  if (linear_drive_scale_map.find("x") != linear_drive_scale_map.end())
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);
  }

  /*if (linear_drive_scale_map.find("y") != linear_drive_scale_map.end())
  {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);
  }

  if (linear_drive_scale_map.find("z") != linear_drive_scale_map.end())
  {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);
  }*/

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(control);

  server->insert(interactive_marker);
  server->setCallback(interactive_marker.name, std::bind(&TwistServerNode::processFeedback, this, std::placeholders::_1));
  server->applyChanges();
}

void TwistServerNode::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
{
  geometry_msgs::msg::Twist vel_msg;

  // Handle angular change (yaw is the only direction in which you can rotate)
  double yaw = tf2::getYaw(feedback->pose.orientation);
  vel_msg.angular.z = angular_drive_scale * yaw;
  vel_msg.angular.z = std::min(vel_msg.angular.z,  max_angular_velocity);
  vel_msg.angular.z = std::max(vel_msg.angular.z, -max_angular_velocity);

  if (linear_drive_scale_map.find("x") != linear_drive_scale_map.end())
  {
    vel_msg.linear.x = linear_drive_scale_map["x"] * feedback->pose.position.x;
    vel_msg.linear.x = std::min(vel_msg.linear.x, max_positive_linear_velocity_map["x"]);
    vel_msg.linear.x = std::max(vel_msg.linear.x, max_negative_linear_velocity_map["x"]);
  }
  
  /*if (linear_drive_scale_map.find("y") != linear_drive_scale_map.end())
  {
    vel_msg.linear.y = linear_drive_scale_map["y"] * feedback->pose.position.y;
    vel_msg.linear.y = std::min(vel_msg.linear.y, max_positive_linear_velocity_map["y"]);
    vel_msg.linear.y = std::max(vel_msg.linear.y, max_negative_linear_velocity_map["y"]);
  }
  
  if (linear_drive_scale_map.find("z") != linear_drive_scale_map.end())
  {
    vel_msg.linear.z = linear_drive_scale_map["z"] * feedback->pose.position.z;
    vel_msg.linear.z = std::min(vel_msg.linear.z, max_positive_linear_velocity_map["z"]);
    vel_msg.linear.z = std::max(vel_msg.linear.z, max_negative_linear_velocity_map["z"]);
  }*/

  vel_pub->publish(vel_msg);

  // Make the marker snap back to robot
  server->setPose(robot_name + "_twist_marker", geometry_msgs::msg::Pose());
  server->applyChanges();
}

} // namespace interactive_marker_twist_server

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interactive_marker_twist_server::TwistServerNode>();
  // Single or Multi?
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}