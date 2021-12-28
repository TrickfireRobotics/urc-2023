#pragma once

#include "rclcpp/rclcpp.hpp"

namespace hello_world
{

class hello_node : public rclcpp::Node
{
public:
  hello_node(const rclcpp::NodeOptions & options);
};

} // namespace hello_world
