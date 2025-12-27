#include "flex_core/FlexCore.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto flex_node = std::make_shared<FlexCore>("flex_control_node");

    rclcpp::spin(flex_node);

    rclcpp::shutdown();
    
    return 0;
}