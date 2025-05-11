# dua_node_cpp

C++ base class library that extends the base `rclcpp::Node` providing direct and easy access to the DUA features.

## Contents

This library provides the new `dua_node::NodeBase` base ROS 2 node class that implements the following features.

- [x] Automatic initialization of an embedded `PManager` object to manage node parameters from the [`params_manager_cpp`](https://github.com/dotX-Automation/params_manager_cpp/blob/master/README.md) library. This way, one only needs to define and call `init_parameters` in the node class constructor to automatically declare and set up the parameters.

## Usage

Just include the `dua_node.hpp` header file in your node class, and inherit from the `dua_node::NodeBase` class. The constructor accepts the following arguments.

- `std::string && node_name`: the name of the node.
- `const rclcpp::NodeOptions & opts`: the node options object, defaults to `rclcpp::NodeOptions()`.
- `verbose`: a boolean flag that enables verbose logs in various utils, defaults to `false`.

---

## Copyright and License

Copyright 2025 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
