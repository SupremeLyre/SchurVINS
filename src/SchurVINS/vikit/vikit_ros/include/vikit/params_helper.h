/*
 * ros_params_helper.h
 *
 *  Created on: Feb 22, 2013
 *      Author: cforster
 *
 * from libpointmatcher_ros
 */

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <string>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>

namespace vk {

using NodePtr = rclcpp::Node::SharedPtr;

namespace detail {

template<typename T>
bool readParameterValue(const rclcpp::Parameter& parameter, T& value)
{
  try
  {
    if constexpr (std::is_floating_point<T>::value)
    {
      if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        value = static_cast<T>(parameter.as_double());
        return true;
      }
      if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        value = static_cast<T>(parameter.as_int());
        return true;
      }
      return false;
    }
    else
    {
      value = parameter.get_value<T>();
      return true;
    }
  }
  catch(const rclcpp::exceptions::InvalidParameterTypeException&)
  {
    return false;
  }
}

} // namespace detail

inline
bool hasParam(const NodePtr& node, const std::string& name)
{
  return node && node->has_parameter(name);
}

inline
bool hasParam(const std::string& name)
{
  RCLCPP_WARN_STREAM(
      rclcpp::get_logger("vikit_params"),
      "ROS2 has no process-wide parameter server; cannot query parameter: " << name);
  return false;
}

template<typename T>
T getParam(const NodePtr& node, const std::string& name, const T& defaultValue)
{
  if(!node)
  {
    RCLCPP_WARN_STREAM(
        rclcpp::get_logger("vikit_params"),
        "Cannot query parameter without a node: " << name << ", assigning default: " << defaultValue);
    return defaultValue;
  }

  T v = defaultValue;
  if(node->has_parameter(name))
  {
    rclcpp::Parameter parameter;
    if(node->get_parameter(name, parameter) && detail::readParameterValue(parameter, v))
    {
      RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << v);
      return v;
    }
    RCLCPP_WARN_STREAM(
        node->get_logger(),
        "Parameter has unexpected type: " << name << ", assigning default: " << defaultValue);
    return defaultValue;
  }

  v = node->declare_parameter<T>(name, defaultValue);
  RCLCPP_WARN_STREAM(
      node->get_logger(),
      "Cannot find value for parameter: " << name << ", assigning default: " << v);
  return v;
}

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
  RCLCPP_WARN_STREAM(
      rclcpp::get_logger("vikit_params"),
      "ROS2 requires node-scoped parameters; assigning default for: " << name
      << ", value: " << defaultValue);
  return defaultValue;
}

template<typename T>
T getParam(const NodePtr& node, const std::string& name)
{
  if(!node)
  {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("vikit_params"),
        "Cannot query parameter without a node: " << name);
    return T();
  }

  T v;
  rclcpp::Parameter parameter;
  if(node->has_parameter(name) &&
     node->get_parameter(name, parameter) &&
     detail::readParameterValue(parameter, v))
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << v);
    return v;
  }

  RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot find value for parameter: " << name);
  return T();
}

template<typename T>
T getParam(const std::string& name)
{
  RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("vikit_params"),
      "ROS2 requires node-scoped parameters; cannot find value for parameter: " << name);
  return T();
}

template<typename T>
T param(const NodePtr& node, const std::string& name, const T& defaultValue,
        const bool silent=false)
{
  if(!node)
  {
    if(!silent)
    {
      RCLCPP_WARN_STREAM(
          rclcpp::get_logger("vikit_params"),
          "Cannot query parameter without a node: " << name << ", assigning default: " << defaultValue);
    }
    return defaultValue;
  }

  if(node->has_parameter(name))
  {
    T v = defaultValue;
    rclcpp::Parameter parameter;
    if(node->get_parameter(name, parameter) && detail::readParameterValue(parameter, v))
    {
      if(!silent)
      {
        RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << v);
      }
      return v;
    }
    if(!silent)
    {
      RCLCPP_WARN_STREAM(
          node->get_logger(),
          "Parameter has unexpected type: " << name << ", assigning default: " << defaultValue);
    }
    return defaultValue;
  }

  const T v = node->declare_parameter<T>(name, defaultValue);
  if(!silent)
  {
    RCLCPP_WARN_STREAM(
        node->get_logger(),
        "Cannot find value for parameter: " << name << ", assigning default: " << v);
  }
  return v;
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
