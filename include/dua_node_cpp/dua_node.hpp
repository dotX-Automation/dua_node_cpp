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

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <dua_qos_cpp/dua_qos.hpp>

#include <params_manager_cpp/params_manager.hpp>

#include <simple_actionclient_cpp/simple_actionclient.hpp>
#include <simple_serviceclient_cpp/simple_serviceclient.hpp>

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
   * @param verbose Verbosity flag.
   */
  NodeBase(
    std::string && node_name,
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

  /**
   * @brief Wraps creation of a timer.
   *        This observes the ROS clock, be it system time or externally supplied.
   *
   * @param name Timer name.
   * @param period Timer period [ms].
   * @param callback Timer callback.
   * @param timer_group Timer callback group.
   * @return Timer shared pointer.
   */
  template<typename CallbackT>
  rclcpp::TimerBase::SharedPtr dua_create_timer(
    const std::string & name,
    unsigned long int period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr timer_cgroup = nullptr)
  {
    rclcpp::TimerBase::SharedPtr timer = create_timer(
      std::chrono::milliseconds(period),
      callback,
      timer_cgroup);
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "[TIMER] '%s' (%ld ms)", name.c_str(), period);
    }
    return timer;
  }

  /**
   * @brief Wraps creation of a wall timer.
   *        This observes the system clock.
   *
   * @param name Timer name.
   * @param period Timer period [ms].
   * @param callback Timer callback.
   * @param timer_group Timer callback group.
   * @return Timer shared pointer.
   */
  template<typename CallbackT>
  rclcpp::TimerBase::SharedPtr dua_create_wall_timer(
    const std::string & name,
    unsigned long int period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr timer_cgroup = nullptr)
  {
    rclcpp::TimerBase::SharedPtr timer = create_wall_timer(
      std::chrono::milliseconds(period),
      callback,
      timer_cgroup);
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "[WALL TIMER] '%s' (%ld ms)", name.c_str(), period);
    }
    return timer;
  }

  /**
   * @brief Wraps the creation of a subscriber.
   *
   * @param topic_name Topic name.
   * @param callback Subscriber callback.
   * @param qos Quality of Service policy.
   * @param sub_cgroup Subscriber callback group.
   * @return Subscriber shared pointer.
   */
  template<typename MessageT, typename CallbackT>
  typename rclcpp::Subscription<MessageT>::SharedPtr dua_create_subscription(
    const std::string & topic_name,
    CallbackT && callback,
    const rclcpp::QoS & qos = dua_qos::Reliable::get_datum_qos(),
    rclcpp::CallbackGroup::SharedPtr sub_cgroup = nullptr)
  {
    typename rclcpp::Subscription<MessageT>::SharedPtr sub = nullptr;
    if (sub_cgroup != nullptr) {
      rclcpp::SubscriptionOptions opts;
      opts.callback_group = sub_cgroup;
      sub = create_subscription<MessageT>(
        topic_name,
        qos,
        std::forward<CallbackT>(callback),
        opts);
    } else {
      sub = create_subscription<MessageT>(
        topic_name,
        qos,
        std::forward<CallbackT>(callback));
    }
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "[TOPIC SUB] '%s'", sub->get_topic_name());
    }
    return sub;
  }

  /**
   * @brief Wraps the creation of a publisher.
   *
   * @param topic_name Topic name.
   * @param qos Quality of Service policy.
   * @return Publisher shared pointer.
   */
  template<typename MessageT>
  typename rclcpp::Publisher<MessageT>::SharedPtr dua_create_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos = dua_qos::Reliable::get_datum_qos())
  {
    typename rclcpp::Publisher<MessageT>::SharedPtr pub =
      create_publisher<MessageT>(topic_name, qos);
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "[TOPIC PUB] '%s'", pub->get_topic_name());
    }
    return pub;
  }

  /**
   * @brief Wraps the creation of a service server.
   *
   * @param service_name Service name.
   * @param callback Service callback.
   * @param srv_cgroup Service callback group.
   * @return Service server shared pointer.
   */
  template<typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr dua_create_service_server(
    const std::string & service_name,
    CallbackT && callback,
    rclcpp::CallbackGroup::SharedPtr srv_cgroup = nullptr)
  {
    typename rclcpp::Service<ServiceT>::SharedPtr server = create_service<ServiceT>(
      service_name,
      std::forward<CallbackT>(callback),
      rclcpp::ServicesQoS(),
      srv_cgroup);
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "[SERVICE SRV] '%s'", server->get_service_name());
    }
    return server;
  }

  /**
   * @brief Wraps the creation of a service client.
   *        This purposely uses the simple_serviceclient::Client implementation.
   *
   * @param service_name Service name.
   * @param wait Whether to wait for the server to become active.
   * @return Service client shared pointer.
   */
  template<typename ServiceT>
  typename simple_serviceclient::Client<ServiceT>::SharedPtr dua_create_service_client(
    const std::string & service_name,
    bool wait = true)
  {
    typename simple_serviceclient::Client<ServiceT>::SharedPtr client =
      std::make_shared<simple_serviceclient::Client<ServiceT>>(
        this,
        service_name,
        wait);
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "[SERVICE CLN] '%s'", client->get_service_name());
    }
    return client;
  }

  /**
   * @brief Wraps the creation of an action server.
   *        Callbacks must always be provided to this.
   *
   * @param action_name Action name.
   * @param goal_callback Goal callback.
   * @param cancel_callback Cancel callback.
   * @param accepted_callback Accepted callback.
   * @param as_cgroup Action server callback group.
   * @return Action server shared pointer.
   */
  template<typename ActionT, typename GoalCallbackT, typename CancelCallbackT, typename AcceptedCallbackT>
  typename rclcpp_action::Server<ActionT>::SharedPtr dua_create_action_server(
    const std::string & action_name,
    GoalCallbackT && goal_callback,
    CancelCallbackT && cancel_callback,
    AcceptedCallbackT && accepted_callback,
    rclcpp::CallbackGroup::SharedPtr as_cgroup = nullptr)
  {
    typename rclcpp_action::Server<ActionT>::SharedPtr server =
      rclcpp_action::create_server<ActionT>(
        this,
        action_name,
        std::forward<GoalCallbackT>(goal_callback),
        std::forward<CancelCallbackT>(cancel_callback),
        std::forward<AcceptedCallbackT>(accepted_callback),
        dua_qos::get_action_server_options(),
        as_cgroup);
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "[ACTION SRV] '%s'", get_entity_fqn(action_name).c_str());
    }
    return server;
  }

  /**
   * @brief Wraps the creation of an action client.
   *        This purposely uses the simple_actionclient::Client implementation.
   *
   * @param action_name Action name.
   * @param feedback_callback Feedback callback.
   * @param wait Whether to wait for the server to become active.
   * @return Action client shared pointer.
   */
  template<typename ActionT>
  typename simple_actionclient::Client<ActionT>::SharedPtr dua_create_action_client(
    const std::string & action_name,
    const simple_actionclient::FeedbackCallbackT<ActionT> & feedback_callback = nullptr,
    bool wait = true)
  {
    typename simple_actionclient::Client<ActionT>::SharedPtr client =
      std::make_shared<simple_actionclient::Client<ActionT>>(
        this,
        action_name,
        feedback_callback,
        wait);
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "[ACTION CLN] '%s'", client->get_action_name().c_str());
    }
    return client;
  }

  /**
   * @brief Returns a mutually exclusive callback group.
   *
   * @return Callback group shared pointer.
   */
  inline rclcpp::CallbackGroup::SharedPtr dua_create_exclusive_cgroup()
  {
    return create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  /**
   * @brief Returns a reentrant callback group.
   *
   * @return Callback group shared pointer.
   */
  inline rclcpp::CallbackGroup::SharedPtr dua_create_reentrant_cgroup()
  {
    return create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }

protected:
  /**
   * @brief Initializes the node calling internal initializers.
   */
  void dua_init_node();

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

  /**
   * @brief Returns the fully qualified name of an entity.
   *
   * @param entity_name Entity name.
   * @return Fully qualified name, may start with a '~'.
   */
  std::string get_entity_fqn(std::string entity_name);

  /* Verbosity flag. */
  bool verbose_ = false;
};

} // namespace dua_node
