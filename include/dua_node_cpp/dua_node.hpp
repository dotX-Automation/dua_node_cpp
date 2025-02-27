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
  NodeBase(std::string && node_name,
    const rclcpp::NodeOptions & opts = rclcpp::NodeOptions(),
    bool verbose = false);
  ~NodeBase();

protected:
  /* Parameter manager object. */
  params_manager::Manager::SharedPtr pmanager_;
};

} // namespace dua_node

#endif // DUA_NODE__DUA_NODE_HPP_
