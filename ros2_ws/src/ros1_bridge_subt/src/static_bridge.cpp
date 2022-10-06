#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <vector>

#include <functional>
#include <utility>

// include ROS 1
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// include ROS 2
#include <rclcpp/rclcpp.hpp>

#include <ros1_bridge/bridge.hpp> //TODO: Change to just bridge

#include <rcl_interfaces/srv/list_parameters.hpp>
#include <ros1_bridge/factory.hpp>
#include <ros1_bridge/factory_interface.hpp>

#define STATIC_BRIDGE_ROS1_QUEUE_SIZE 4000

using namespace std::chrono_literals;

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode(const std::string& name = "ros_bridge_subt",
             const std::string& namespace_ = "",
             const rclcpp::NodeOptions& options =
                 (rclcpp::NodeOptions()
                      .allow_undeclared_parameters(true)
                      .automatically_declare_parameters_from_overrides(true)))
    : rclcpp::Node(name, namespace_, options) {
    RCLCPP_INFO(
        this->get_logger(),
        "Parameter in node named '%s' ready, "
        "and serving '%zu' parameters already!",
        this->get_fully_qualified_name(),
        this->list_parameters(
                {},
                rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE)
            .names.size());
  }
};

ros1_bridge::Bridge1to2Handles
create_bridge_from_1_to_2_qos(ros::NodeHandle ros1_node,
                              const std::string& ros1_type_name,
                              const std::string& ros1_topic_name,
                              size_t subscriber_queue_size,
                              rclcpp::Node::SharedPtr ros2_node,
                              const std::string& ros2_type_name,
                              const std::string& ros2_topic_name,
                              const rmw_qos_profile_t& qos_profile) {
  auto factory = ros1_bridge::get_factory(ros1_type_name, ros2_type_name);
  auto ros2_pub = factory->create_ros2_publisher(ros2_node,
                                                 ros2_topic_name,
                                                 qos_profile);
  auto ros1_sub = factory->create_ros1_subscriber(ros1_node,
                                                  ros1_topic_name,
                                                  subscriber_queue_size,
                                                  ros2_pub,
                                                  ros2_node->get_logger());

  ros1_bridge::Bridge1to2Handles handles;
  handles.ros1_subscriber = ros1_sub;
  handles.ros2_publisher = ros2_pub;
  return handles;
}

ros1_bridge::Bridge2to1Handles
create_bridge_from_2_to_1_qos(rclcpp::Node::SharedPtr ros2_node,
                              const std::string& ros2_type_name,
                              const std::string& ros2_topic_name,
                              const rmw_qos_profile_t& qos_profile,
                              ros::NodeHandle ros1_node,
                              const std::string& ros1_type_name,
                              const std::string& ros1_topic_name,
                              size_t publisher_queue_size) {
  auto factory = ros1_bridge::get_factory(ros1_type_name, ros2_type_name);
  auto ros1_pub = factory->create_ros1_publisher(
      ros1_node, ros1_topic_name, publisher_queue_size);

  auto ros2_sub = factory->create_ros2_subscriber(
      ros2_node, ros2_topic_name, qos_profile, ros1_pub);

  ros1_bridge::Bridge2to1Handles handles;
  handles.ros2_subscriber = ros2_sub;
  handles.ros1_publisher = ros1_pub;
  return handles;
}
#if 0 // This code is unused.
ros1_bridge::BridgeHandles
create_bidirectional_bridge_qos(ros::NodeHandle ros1_node,
                                rclcpp::Node::SharedPtr ros2_node,
                                const std::string& ros1_type_name,
                                const std::string& ros2_type_name,
                                const std::string& topic_name,
                                size_t queue_size,
                                const rmw_qos_profile_t& qos_profile) {
  RCLCPP_INFO(ros2_node->get_logger(),
              "create bidirectional bridge for topic " + topic_name);
  ros1_bridge::BridgeHandles handles;
  handles.bridge1to2 = create_bridge_from_1_to_2_qos(ros1_node,
                                                     ros2_node,
                                                     ros1_type_name,
                                                     topic_name,
                                                     queue_size,
                                                     ros2_type_name,
                                                     topic_name,
                                                     queue_size,
                                                     qos_profile);
  handles.bridge2to1 = create_bridge_from_2_to_1_qos(ros2_node,
                                                     ros1_node,
                                                     ros2_type_name,
                                                     topic_name,
                                                     queue_size,
                                                     ros1_type_name,
                                                     topic_name,
                                                     queue_size,
                                                     handles.bridge1to2.ros2_publisher,
                                                     qos_profile);
  return handles;
}
#endif

rmw_qos_profile_t read_qos_from_param(
    std::shared_ptr<rclcpp::Node> node,
    const std::string &param_prefix) {

  std::string param_name;

  // Set queue size
  size_t qos_queuesize = 10;
  if (!node->get_parameter(param_prefix + ".queue_size", qos_queuesize)) {
    RCLCPP_INFO(node->get_logger(),
                 "Missing parameter: %s.queue_size. Using default value", param_prefix);
  }

  // Set history
  std::string qos_history;
  rmw_qos_history_policy_t history_policy;
  if (!node->get_parameter(param_prefix + ".history", qos_history)) {
    RCLCPP_INFO(node->get_logger(),
                 "Missing parameter: %s.history. Using default value", param_prefix);
    history_policy = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
  } else {
    if (qos_history == "KEEP_ALL") {
      history_policy = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    } else if (qos_history == "KEEP_LAST") {
      history_policy = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    } else {
      RCLCPP_ERROR(node->get_logger(),
                   "History parameter of %s is not recognized",
                   param_prefix);
    }
  }

  // Set durability
  std::string qos_durability;
  rmw_qos_durability_policy_t durability_policy;
  if (!node->get_parameter(param_prefix + ".durability", qos_durability)) {
    RCLCPP_INFO(node->get_logger(),
                 "Missing parameter: %s.durability. Using default value", param_prefix);
    durability_policy = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
  } else {
    if (qos_durability == "VOLATILE") {
      durability_policy = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    } else if (qos_durability == "TRANSIENT_LOCAL") {
      durability_policy = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    } else {
      RCLCPP_ERROR(node->get_logger(),
                   "Durability parameter of %s is not recognized",
                   param_prefix);
    }
  }

  // Set reliability
  std::string qos_reliability;
  rmw_qos_reliability_policy_t reliability_policy;
  if (!node->get_parameter(param_prefix + ".reliability", qos_reliability)) {
    RCLCPP_INFO(node->get_logger(),
                 "Missing parameter: %s.reliability. Using default value", param_prefix);
    reliability_policy = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  } else {
    if (qos_reliability == "RELIABLE") {
      reliability_policy = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    } else if (qos_reliability == "BEST_EFFORT") {
      reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    } else {
      RCLCPP_ERROR(node->get_logger(),
                   "Reliability parameter of %s is not recognized",
                   param_prefix);
    }
  }

  // Set deadline
  rmw_time_t qos_deadline = {0, 0}; //sec, nsec
  float deadline_param_value = 0;
  if (!node->get_parameter(param_prefix + ".deadline", deadline_param_value)) {
    RCLCPP_INFO(node->get_logger(),
                 "Missing parameter: %s.deadline. Using default value", param_prefix.c_str());
  }
  ros::Duration deadline_ros_duration(deadline_param_value);
  qos_deadline.sec = deadline_ros_duration.sec;
  qos_deadline.nsec = deadline_ros_duration.nsec;

  // Set lifespan
  rmw_time_t qos_lifespan = {0, 0}; //sec, nsec
  float lifespan_param_value = 0;
  if (!node->get_parameter(param_prefix + ".lifespan", lifespan_param_value)) {
    RCLCPP_INFO(node->get_logger(),
                 "Missing parameter: %s.lifespan. Using default value", param_prefix.c_str());
  }
  ros::Duration lifespan_ros_duration(lifespan_param_value);
  qos_lifespan.sec = lifespan_ros_duration.sec;
  qos_lifespan.nsec = lifespan_ros_duration.nsec;

  // Set liveliness policy
  std::string qos_liveliness;
  rmw_qos_liveliness_policy_t liveliness_policy;
  if (!node->get_parameter(param_prefix + ".liveliness", qos_liveliness)) {
    RCLCPP_INFO(node->get_logger(),
                 "Missing parameter: %s.liveliness. Using default value", param_prefix.c_str());
    liveliness_policy = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
  } else {
    if (qos_liveliness == "SYSTEM_DEFAULT") {
      liveliness_policy = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    } else if (qos_liveliness == "AUTOMATIC") {
      liveliness_policy = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    } else if (qos_liveliness == "MANUAL_BY_NODE") {
      liveliness_policy = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE;
    } else {
      RCLCPP_ERROR(node->get_logger(),
                   "Liveliness parameter of %s is not recognized",
                   param_prefix.c_str());
    }
  }

  // Set liveliness lease duration
  rmw_time_t qos_liveliness_duration = {0, 0}; //sec, nsec
  float liveliness_duration_param_value = 0.0;
  if (!node->get_parameter(param_prefix + ".liveliness_duration", liveliness_duration_param_value)) {
    RCLCPP_INFO(node->get_logger(),
                 "Missing parameter: %s.liveliness_duration. Using default value", param_prefix.c_str());
  }
  ros::Duration liveliness_ros_duration(liveliness_duration_param_value);
  qos_liveliness_duration.sec = liveliness_ros_duration.sec;
  qos_liveliness_duration.nsec = liveliness_ros_duration.nsec;


  // Create QoS profile
  rmw_qos_profile_t qos_profile = {
      history_policy,
      qos_queuesize,
      reliability_policy,
      durability_policy,
      qos_deadline,
      qos_lifespan,
      liveliness_policy,
      qos_liveliness_duration,
      false};

  return qos_profile;
}


std::vector<ros1_bridge::BridgeHandles> _create_bridge(
    ros::NodeHandle ros1_node,
    std::shared_ptr<rclcpp::Node> ros2_node,
    const std::string &param_prefix,
    bool to_ros2 = true) {

  // Get robot and base list
  std::vector<std::string> robot_prefix;
  if (!ros2_node->get_parameter("robots", robot_prefix)) {
    RCLCPP_WARN(ros2_node->get_logger(),
                "No robot list in the yaml file");
  }
  else {
    RCLCPP_INFO(ros2_node->get_logger(),
                "Substitution keyword '%s' found", "robots");
  }

  std::vector<std::string> base_prefix;
  if (!ros2_node->get_parameter("substitutions.base", base_prefix)) {
    RCLCPP_WARN(ros2_node->get_logger(),
                "No base list in the yaml file");
  }
  else {
    RCLCPP_INFO(ros2_node->get_logger(),
                "Substitution keyword '%s' found", "base");
  }
  RCLCPP_INFO(ros2_node->get_logger(),
              "this namespace '%s'", ros1_node.getNamespace().c_str());

  // Get topic list to be mapped
  auto topics_prefixes = ros2_node->list_parameters({param_prefix}, 3);

  std::vector<ros1_bridge::BridgeHandles> bridgehandles;
  for (auto& topic_iter : topics_prefixes.prefixes) {
    if (topic_iter == param_prefix)
      continue; // Ignore prefix "topics" without the child name

    // Read topic names
    std::vector<std::string> topic_names;
    if (!ros2_node->get_parameter(topic_iter + ".topic_names", topic_names)) {
      RCLCPP_WARN(ros2_node->get_logger(),
                  "No defined topic names in the yaml file");
    }

    // Read topic types
    std::string ros1_type_name;
    if (!ros2_node->get_parameter(topic_iter + ".ros1_type_name", ros1_type_name)) {
      RCLCPP_WARN(ros2_node->get_logger(),
                  "No defined ros1 topic type in the yaml file");
    }
    std::string ros2_type_name;
    if (!ros2_node->get_parameter(topic_iter + ".ros2_type_name", ros2_type_name)) {
      RCLCPP_WARN(ros2_node->get_logger(),
                  "No defined ros2 topic type in the yaml file");
    }

    // Setup the QoS
    auto qos_profile = read_qos_from_param(ros2_node, topic_iter);

    // Process substitition keywords for robot names
    //Substitution for all_robots
    std::vector<std::string> canonical_topic_names;
    std::string keyword_all_robots = "_ALL_ROBOTS_";
    std::string keyword_other_robots = "_OTHER_ROBOTS_";
    std::string keyword_base = "_BASE_";

    for (auto& topic_name : topic_names) {
      if (topic_name.find(keyword_all_robots) != std::string::npos) {
        for (auto& namespace_iter : robot_prefix) {
          canonical_topic_names.push_back(
            ros1_node.resolveName(
              std::regex_replace(topic_name, std::regex(keyword_all_robots), namespace_iter)
            )
          );
        }
      } else if (topic_name.find(keyword_other_robots) != std::string::npos) {
        for (auto& namespace_iter : robot_prefix) {
          if(namespace_iter != ros1_node.getNamespace().substr(1)) {
          canonical_topic_names.push_back(
            ros1_node.resolveName(
              std::regex_replace(topic_name, std::regex(keyword_other_robots), namespace_iter)
            )
          );
          }
        }
      } else if (topic_name.find(keyword_base) != std::string::npos) {
        for (auto& namespace_iter : base_prefix) {
          canonical_topic_names.push_back(
            ros1_node.resolveName(
              std::regex_replace(topic_name, std::regex(keyword_base), namespace_iter)
            )
          );
        }
      } else {
        canonical_topic_names.push_back(ros1_node.resolveName(topic_name));
      }
    }

    // Create bridges
    for (auto& topic_name : canonical_topic_names) {
      ros1_bridge::BridgeHandles handle;
      if (to_ros2) {
        handle.bridge1to2 =
            create_bridge_from_1_to_2_qos(ros1_node,
                                          ros1_type_name,
                                          topic_name,
                                          STATIC_BRIDGE_ROS1_QUEUE_SIZE,
                                          ros2_node,
                                          ros2_type_name,
                                          topic_name,
                                          qos_profile);
        RCLCPP_INFO(ros2_node->get_logger(),
                    "Create 1->2 bridge for topic " + topic_name);
      } else {
        handle.bridge2to1 =
            create_bridge_from_2_to_1_qos(ros2_node,
                                          ros2_type_name,
                                          topic_name,
                                          qos_profile,
                                          ros1_node,
                                          ros1_type_name,
                                          topic_name,
                                          STATIC_BRIDGE_ROS1_QUEUE_SIZE);
        RCLCPP_INFO(ros2_node->get_logger(),
                    "Create 2->1 bridge for topic " + topic_name);
      }
      bridgehandles.push_back(handle);
    }
  } // End loop for each topic in parameter
  return bridgehandles;
}

std::vector<ros1_bridge::BridgeHandles> create_bridge(
    std::shared_ptr<rclcpp::Node> ros2_node,
    ros::NodeHandle ros1_node,
    const std::string &param_prefix) {
  return _create_bridge(ros1_node, ros2_node, param_prefix, false);
}

std::vector<ros1_bridge::BridgeHandles> create_bridge(
    ros::NodeHandle ros1_node,
    std::shared_ptr<rclcpp::Node> ros2_node,
    const std::string &param_prefix) {
  return _create_bridge(ros1_node, ros2_node, param_prefix, true);
}

int main(int argc, char* argv[]) {
  // Init ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = std::make_shared<BridgeNode>();

  // Init ROS 1 node
  // NOTE: Right now ROS2 Node need to be declared first
  //       before ROS1 node belows
  ros::init(argc, argv, "ros_bridge_subt");
  ros::NodeHandle ros1_node;

  // Create inbound/outbound bridges
  auto handles_1to2 = create_bridge(ros1_node, ros2_node, "topics_1_to_2");
  auto handles_2to1 = create_bridge(ros2_node, ros1_node, "topics_2_to_1");

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
  }

  return 0;
}
