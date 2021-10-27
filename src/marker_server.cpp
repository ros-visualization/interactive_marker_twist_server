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

namespace interactive_marker_twist_server
{

class TwistServerNode : public rclcpp::Node
{
public:
  TwistServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~TwistServerNode() = default;

private:
  void createInteractiveMarkers();
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;
}; // class TwistServerNode

TwistServerNode::TwistServerNode(const rclcpp::NodeOptions & options) : rclcpp::Node("twist_server_node", options)
{
  vel_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  server = std::make_unique<interactive_markers::InteractiveMarkerServer>("twist_server", get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(), get_node_topics_interface(), get_node_services_interface());
  createInteractiveMarkers();
  RCLCPP_INFO(get_logger(), "Interactive Marker Twist Server Ready.");
}

void TwistServerNode::createInteractiveMarkers()
{
  visualization_msgs::msg::InteractiveMarker interactive_marker;
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

  interactive_marker.controls.push_back(rotate_control);

  server->insert(interactive_marker);
  server->setCallback(interactive_marker.name, std::bind(&TwistServerNode::processFeedback, this, std::placeholders::_1));
  server->applyChanges();
}

void TwistServerNode::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
{
  geometry_msgs::msg::Twist vel_msg;

  vel_msg.angular.z = 0.5;
  vel_msg.linear.x = 1.2;

  vel_pub->publish(vel_msg);

  RCLCPP_INFO(get_logger(), "Publishing: '%f'", vel_msg.linear.x);

  // Make the marker snap back to robot
  //server->setPose(robot_name + "_twist_marker", geometry_msgs::msg::Pose());

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