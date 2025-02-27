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

/**
 * Constructor.
 *
 * @param node_name Name of the node.
 * @param opts Node options.
 */
NodeBase::NodeBase(
  std::string && node_name,
  const rclcpp::NodeOptions & opts,
  bool verbose)
: Node(node_name, opts)
{
  // Create and initialize Parameter Manager object
  pmanager_ = std::make_shared<params_manager::Manager>(this, verbose);
}

/**
 * Destructor.
 */
NodeBase::~NodeBase()
{
  // Destroy Parameter Manager object
  pmanager_.reset();
}

} // namespace dua_node
