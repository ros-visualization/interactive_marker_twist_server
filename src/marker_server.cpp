/*
 * Copyright (c) 2011, Willow Garage, Inc.
 *           (c) 2013, Mike Purvis
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
  TwistServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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

TwistServerNode::TwistServerNode(const rclcpp::NodeOptions & options) : rclcpp::Node("twist_server_node", options)
{
  getParameters();
  vel_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  server = std::make_unique<interactive_markers::InteractiveMarkerServer>("twist_server", get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(), get_node_topics_interface(), get_node_services_interface());
  createInteractiveMarkers();
  RCLCPP_INFO(get_logger(), "[interacive_marker_twist_server] Initialized.");
}

void TwistServerNode::getParameters()
{
  this->declare_parameter<std::string>("link_name", "base_link");
  //nh.param<std::string>("link_name", link_name, "base_link");
  this->declare_parameter<std::string>("robot_name", "robot");
  //nh.param<std::string>("robot_name", robot_name, "robot");

  //this->get_parameter("linear_scale", linear_drive_scale_map);
  /*if (this->get_parameter("linear_scale", linear_drive_scale_map))
  //if (nh.getParam("linear_scale", linear_drive_scale_map))
  {
    //nh.getParam("linear_scale", linear_drive_scale_map);
    this->get_parameter("linear_scale", linear_drive_scale_map);
    //nh.getParam("max_positive_linear_velocity", max_positive_linear_velocity_map);
    this->get_parameter("max_positive_linear_velocity", max_positive_linear_velocity_map);
    //nh.getParam("max_negative_linear_velocity", max_negative_linear_velocity_map);
    this->get_parameter("max_negative_linear_velocity", max_negative_linear_velocity_map);
  }
  else
  {
    this->declare_parameter<double>("linear_scale", 1.0);
    this->declare_parameter<double>("max_positive_linear_velocity", 1.0);
    this->declare_parameter<double>("max_negative_linear_velocity", -1.0);
    //nh.param<double>("linear_scale", linear_drive_scale_map["x"], 1.0);
    //nh.param<double>("max_positive_linear_velocity", max_positive_linear_velocity_map["x"],  1.0);
    //nh.param<double>("max_negative_linear_velocity", max_negative_linear_velocity_map["x"], -1.0);
  }

  this->declare_parameter<double>("angular_scale", 2.2);
  this->declare_parameter<double>("max_angular_velocity", 2.2);
  this->declare_parameter<double>("marker_size_scale", 1.0);
  //nh.param<double>("angular_scale", angular_drive_scale, 2.2);
  //nh.param<double>("max_angular_velocity", max_angular_velocity, 2.2);
  //nh.param<double>("marker_size_scale", marker_size_scale, 1.0);*/
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
  if (linear_drive_scale_map.find("y") != linear_drive_scale_map.end())
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
  }

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(control);

  /*visualization_msgs::msg::InteractiveMarker interactive_marker;
  interactive_marker.header.frame_id = "base_link";
  interactive_marker.header.stamp = get_clock()->now();
  interactive_marker.name = "my_marker";
  interactive_marker.description = "Simple 1-DOF Control";

  visualization_msgs::msg::Marker box_marker;
  box_marker.type = visualization_msgs::msg::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);

  interactive_marker.controls.push_back(box_control);

  visualization_msgs::msg::InteractiveMarkerControl rotate_control;
  rotate_control.name = "move_x";
  rotate_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

  interactive_marker.controls.push_back(rotate_control);*/

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
  if (linear_drive_scale_map.find("y") != linear_drive_scale_map.end())
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
  }

  vel_pub->publish(vel_msg);

  RCLCPP_INFO(get_logger(), "Publishing: '%f'", vel_msg.linear.x);
  RCLCPP_INFO(get_logger(), "Publishing: '%f'", vel_msg.linear.y);
  RCLCPP_INFO(get_logger(), "Publishing: '%f'", vel_msg.linear.z);
  RCLCPP_INFO(get_logger(), "Publishing: '%f'", vel_msg.angular.z);

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