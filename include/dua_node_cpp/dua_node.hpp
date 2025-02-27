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

#ifndef DUA_NODE__DUA_NODE_HPP_
#define DUA_NODE__DUA_NODE_HPP_

#include "visibility_control.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <params_manager/params_manager.hpp>

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
  /* Parameter manager object. */
  params_manager::Manager::SharedPtr pmanager_;
};

} // namespace dua_node

#endif // DUA_NODE__DUA_NODE_HPP_
