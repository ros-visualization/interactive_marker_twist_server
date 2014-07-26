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

#include "interactive_marker_twist_server/marker_server.h"
#include "interactive_markers/interactive_marker_server.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "tf/tf.h"

#include <algorithm>
#include <string>

using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;
using interactive_markers::InteractiveMarkerServer;

namespace interactive_marker_twist_server
{

struct MarkerServer::Impl
{
  void processFeedback(const InteractiveMarkerFeedback::ConstPtr &feedback);
  void createMarkers();

  ros::Publisher vel_pub;
  boost::scoped_ptr<InteractiveMarkerServer> server;

  double linear_drive_scale;
  double angular_drive_scale;
  double max_positive_linear_velocity;
  double max_negative_linear_velocity;
  double max_angular_velocity;
  double marker_size_scale;

  std::string link_name;
  std::string robot_name;
};

MarkerServer::MarkerServer(ros::NodeHandle* nh, ros::NodeHandle* nh_param,
                           const std::string& server_name)
{
  pimpl_ = new Impl;
  pimpl_->server.reset(new InteractiveMarkerServer(server_name, server_name));

  nh_param->param<std::string>("link_name", pimpl_->link_name, "base_link");
  nh_param->param<std::string>("robot_name", pimpl_->robot_name, "robot");

  nh_param->param<double>("linear_scale", pimpl_->linear_drive_scale, 1.0);
  nh_param->param<double>("angular_scale", pimpl_->angular_drive_scale, 2.2);
  nh_param->param<double>("marker_size_scale", pimpl_->marker_size_scale, 1.0);

  nh_param->param<double>("max_positive_linear_velocity", pimpl_->max_positive_linear_velocity, 1.0);
  nh_param->param<double>("max_negative_linear_velocity", pimpl_->max_negative_linear_velocity, -1.0);
  nh_param->param<double>("max_angular_velocity", pimpl_->max_angular_velocity, 2.2);

  pimpl_->vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pimpl_->createMarkers();

  ROS_INFO_NAMED(server_name, "Initialization complete.");
}

MarkerServer::~MarkerServer()
{
  delete pimpl_;
}

void MarkerServer::Impl::processFeedback(const InteractiveMarkerFeedback::ConstPtr& feedback)
{
  geometry_msgs::Twist vel;

  if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE)
  {
    // Handle angular change (yaw is the only direction in which you can rotate)
    double yaw = tf::getYaw(feedback->pose.orientation);
    vel.angular.z = angular_drive_scale * yaw;
    vel.linear.x = linear_drive_scale * feedback->pose.position.x;

    // Enforce parameterized speed limits
    vel.linear.x = std::min(vel.linear.x, max_positive_linear_velocity);
    vel.linear.x = std::max(vel.linear.x, max_negative_linear_velocity);
    vel.angular.z = std::min(vel.angular.z, max_angular_velocity);
    vel.angular.z = std::max(vel.angular.z, -max_angular_velocity);
  }

  // The MOUSE_UP event triggers a zero-velocity message.
  vel_pub.publish(vel);

  // Snap the marker back to the robot.
  server->setPose(robot_name + "_twist_marker", geometry_msgs::Pose());
  server->applyChanges();
}

void MarkerServer::Impl::createMarkers()
{
  // create an interactive marker for our server
  InteractiveMarker int_marker;
  int_marker.header.frame_id = link_name;
  int_marker.name = robot_name + "_twist_marker";
  int_marker.description = "twist controller for " + robot_name;
  int_marker.scale = marker_size_scale;

  InteractiveMarkerControl control;

  control.orientation_mode = InteractiveMarkerControl::FIXED;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);

  server->insert(int_marker, boost::bind(&MarkerServer::Impl::processFeedback, this, _1));
  server->applyChanges();
}

}  // namespace interactive_marker_twist_server
