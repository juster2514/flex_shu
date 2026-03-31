#include "flex_core/FlexPathCore.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FlexPathCore>("flex_path_core");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

