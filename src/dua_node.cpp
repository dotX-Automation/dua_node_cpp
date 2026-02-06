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

void NodeBase::dua_init_node()
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
    RCLCPP_INFO(get_logger(), "--- PARAMETERS ---");
  }

  // Declare TF server parameters
  pmanager_->declare_bool_parameter(
    "dua.tf_server.get_transform",
    false,
    "Enable GetTransform client, allows use of standard get_transform API.",
    "Must remap the service name to a compliant, existing service.",
    true,
    &tf_server_get_transform_);
  pmanager_->declare_bool_parameter(
    "dua.tf_server.transform_pose",
    false,
    "Enable TransformPose client, allows use of standard transform_pose API.",
    "Must remap the service name to a compliant, existing service.",
    true,
    &tf_server_transform_pose_);
  pmanager_->declare_bool_parameter(
    "dua.tf_server.wait_servers",
    false,
    "Wait for TF servers to be available during node initialization.",
    "If enabled, the node initialization will block until the TF servers are available.",
    true,
    &tf_server_wait_servers_);

  // Declare the rest of the parameters
  init_parameters();
}

void NodeBase::dua_init_cgroups()
{
  init_cgroups();
}

void NodeBase::dua_init_timers()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "--- TIMERS ---");
  }
  init_timers();
}

void NodeBase::dua_init_subscribers()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "--- SUBSCRIBERS ---");
  }
  init_subscribers();
}

void NodeBase::dua_init_publishers()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "--- PUBLISHERS ---");
  }
  init_publishers();
}

void NodeBase::dua_init_service_servers()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "--- SERVICE SERVERS ---");
  }
  init_service_servers();
}

void NodeBase::dua_init_service_clients()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "--- SERVICE CLIENTS ---");
  }

  // Initialize TF server clients
  // get_transform
  if (tf_server_get_transform_) {
    get_transform_client_ = dua_create_service_client<dua_geometry_interfaces::srv::GetTransform>(
      "/get_transform",
      tf_server_wait_servers_);
  }

  // transform_pose
  if (tf_server_transform_pose_) {
    transform_pose_client_ = dua_create_service_client<dua_geometry_interfaces::srv::TransformPose>(
      "/transform_pose",
      tf_server_wait_servers_);
  }

  // Initialize the rest of the clients
  init_service_clients();
}

void NodeBase::dua_init_action_servers()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "--- ACTION SERVERS ---");
  }
  init_action_servers();
}

void NodeBase::dua_init_action_clients()
{
  if (verbose_) {
    RCLCPP_INFO(get_logger(), "--- ACTION CLIENTS ---");
  }
  init_action_clients();
}

uint8_t NodeBase::get_transform(
  const std_msgs::msg::Header & source,
  const std_msgs::msg::Header & target,
  geometry_msgs::msg::TransformStamped & transform,
  bool transform_frames,
  const rclcpp::Duration & timeout,
  bool spin,
  int64_t srv_timeout)
{
  // Consistency check
  if (get_transform_client_== nullptr) {
    throw std::runtime_error("dua_node::NodeBase::get_transform: client not initialized");
  }

  // Create the request
  auto req = std::make_shared<dua_geometry_interfaces::srv::GetTransform::Request>();
  if (transform_frames) {
    req->set__source(target);
    req->set__target(source);
  } else {
    req->set__source(source);
    req->set__target(target);
  }
  req->set__timeout(timeout);

  // Send the request
  auto resp = get_transform_client_->call_sync(req, spin, srv_timeout);

  // Check the response
  if (resp == nullptr) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "GetTransform call error ('%s' -> '%s'): no response",
      source.frame_id.c_str(),
      target.frame_id.c_str());
    return dua_common_interfaces::msg::CommandResultStamped::TIMEOUT;
  }
  if (resp->result.result == dua_common_interfaces::msg::CommandResultStamped::ERROR) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "GetTransform server error ('%s' -> '%s'): %s",
      source.frame_id.c_str(),
      target.frame_id.c_str(),
      resp->result.error_msg.c_str());
    return resp->result.result;
  }

  // Return the transform and the operation result
  transform = resp->transform;
  return resp->result.result;
}

uint8_t NodeBase::transform_pose(
  const geometry_msgs::msg::PoseStamped & source_pose,
  const std_msgs::msg::Header & target,
  geometry_msgs::msg::PoseStamped & target_pose,
  const rclcpp::Duration & timeout,
  bool spin,
  int64_t srv_timeout)
{
  // Consistency check
  if (transform_pose_client_ == nullptr) {
    throw std::runtime_error("dua_node::NodeBase::transform_pose: client not initialized");
  }

  // Create the request
  auto req = std::make_shared<dua_geometry_interfaces::srv::TransformPose::Request>();
  req->set__source_pose(source_pose);
  req->set__target(target);
  req->set__timeout(timeout);

  // Send the request
  auto resp = transform_pose_client_->call_sync(req, spin, srv_timeout);

  // Check the response
  if (resp == nullptr) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TransformPose call error ('%s' -> '%s'): no response",
      source_pose.header.frame_id.c_str(),
      target.frame_id.c_str());
    return dua_common_interfaces::msg::CommandResultStamped::TIMEOUT;
  }
  if (resp->result.result == dua_common_interfaces::msg::CommandResultStamped::ERROR) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TransformPose server error ('%s' -> '%s'): %s",
      source_pose.header.frame_id.c_str(),
      target.frame_id.c_str(),
      resp->result.error_msg.c_str());
    return resp->result.result;
  }

  // Return the result
  target_pose = resp->target_pose;
  return resp->result.result;
}

std::string NodeBase::get_entity_fqn(std::string entity_name)
{
  std::string ns = std::string(get_fully_qualified_name());
  size_t pos = entity_name.find_last_of("/");
  if (pos != std::string::npos) {
    entity_name = entity_name.substr(pos + 1);
  }
  return ns + "/" + entity_name;
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
