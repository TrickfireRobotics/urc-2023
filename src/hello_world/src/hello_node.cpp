#include "hello_world/hello_node.hpp"

namespace hello_world
{

hello_node::hello_node(const rclcpp::NodeOptions& options)
: Node("hello_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Hello World!");
}

} // namespace hello_world

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hello_world::hello_node)