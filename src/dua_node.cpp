/**
 * DUA ROS 2 node base class implementation.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
 *
 * July 10, 2024
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <dua_node_cpp/dua_node.hpp>

namespace dua_node
{

NodeBase::NodeBase(
  std::string && node_name,
  const rclcpp::NodeOptions & opts,
  bool verbose)
: Node(node_name, opts),
  verbose_(verbose)
{
  // Create and initialize Parameter Manager object
  pmanager_ = std::make_shared<params_manager::Manager>(this, verbose);
}

NodeBase::~NodeBase()
{
  // Destroy Parameter Manager object
  pmanager_.reset();
}

void NodeBase::init_node()
{
  dua_init_parameters();
  dua_init_cgroups();
  dua_init_timers();
  dua_init_subscribers();
  dua_init_publishers();
  dua_init_service_servers();
  dua_init_service_clients();
  dua_init_action_servers();
  dua_init_action_clients();
}

void NodeBase::dua_init_parameters()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "Parameters:");
  }
  init_parameters();
}

void NodeBase::dua_init_cgroups()
{
  init_cgroups();
}

void NodeBase::dua_init_timers()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "Timers:");
  }
  init_timers();
}

void NodeBase::dua_init_subscribers()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "Subscribers:");
  }
  init_subscribers();
}

void NodeBase::dua_init_publishers()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "Publishers:");
  }
  init_publishers();
}

void NodeBase::dua_init_service_servers()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "Service servers:");
  }
  init_service_servers();
}

void NodeBase::dua_init_service_clients()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "Service clients:");
  }
  init_service_clients();
}

void NodeBase::dua_init_action_servers()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "Action servers:");
  }
  init_action_servers();
}

void NodeBase::dua_init_action_clients()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "Action clients:");
  }
  init_action_clients();
}

NodeBase::SharedPtr NodeBase::shared_from_this()
{
  return std::static_pointer_cast<NodeBase>(rclcpp::Node::shared_from_this());
}

NodeBase::ConstSharedPtr NodeBase::shared_from_this() const
{
  return std::static_pointer_cast<const NodeBase>(rclcpp::Node::shared_from_this());
}

} // namespace dua_node
