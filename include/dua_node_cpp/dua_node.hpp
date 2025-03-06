/**
 * DUA ROS 2 node base class.
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

#pragma once

#include "visibility_control.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <params_manager_cpp/params_manager.hpp>

namespace dua_node
{

/**
 * Extends rclcpp::Node adding features from DUA packages.
 */
class DUA_NODE_PUBLIC NodeBase : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<NodeBase>;
  using WeakPtr = std::weak_ptr<NodeBase>;
  using UniquePtr = std::unique_ptr<NodeBase>;
  using ConstSharedPtr = std::shared_ptr<const NodeBase>;
  using ConstWeakPtr = std::weak_ptr<const NodeBase>;

  /**
   * Constructor.
   *
   * @param node_name Name of the node.
   * @param opts Node options.
   */
  NodeBase(std::string && node_name,
    const rclcpp::NodeOptions & opts = rclcpp::NodeOptions(),
    bool verbose = false);

  /**
   * Destructor.
   */
  ~NodeBase();

  /**
   * @brief Get a shared_ptr to this instance.
   *
   * @return Shared pointer to this instance.
   */
  NodeBase::SharedPtr shared_from_this();

  /**
   * @brief Get a const shared_ptr to this instance.
   *
   * @return Const shared pointer to this instance.
   */
  NodeBase::ConstSharedPtr shared_from_this() const;

protected:
  /**
   * @brief Initializes the node calling internal initializers.
   */
  void init_node();

  /**
   * @brief Initializes the node parameters (must be overridden).
   */
  virtual void init_parameters()
  {}

  /**
   * @brief Initializes the node callback groups (must be overridden).
   */
  virtual void init_cgroups()
  {}

  /**
   * @brief Initializes the node timers (must be overridden).
   */
  virtual void init_timers()
  {}

  /**
   * @brief Initializes the node subscribers (must be overridden).
   */
  virtual void init_subscribers()
  {}

  /**
   * @brief Initializes the node publishers (must be overridden).
   */
  virtual void init_publishers()
  {}

  /**
   * @brief Initializes the node service servers (must be overridden).
   */
  virtual void init_service_servers()
  {}

  /**
   * @brief Initializes the node service clients (must be overridden).
   */
  virtual void init_service_clients()
  {}

  /**
   * @brief Initializes the node action servers (must be overridden).
   */
  virtual void init_action_servers()
  {}

  /**
   * @brief Initializes the node action clients (must be overridden).
   */
  virtual void init_action_clients()
  {}

  /* Parameter manager object. */
  params_manager::Manager::SharedPtr pmanager_;

private:
    /**
     * @brief Initializes node parameters.
     */
    void dua_init_parameters();

    /**
     * @brief Initializes callback groups.
     */
    void dua_init_cgroups();

    /**
     * @brief Initializes timers.
     */
    void dua_init_timers();

    /**
     * @brief Initializes subscribers.
     */
    void dua_init_subscribers();

    /**
     * @brief Initializes publishers.
     */
    void dua_init_publishers();

    /**
     * @brief Initializes service servers.
     */
    void dua_init_service_servers();

    /**
     * @brief Initializes service clients.
     */
    void dua_init_service_clients();

    /**
     * @brief Initializes actions servers.
     */
    void dua_init_action_servers();

    /**
     * @brief Initializes actions clients.
     */
    void dua_init_action_clients();

    /* Verbosity flag. */
    bool verbose_ = false;
};

} // namespace dua_node
