#include "flex_core/RemoteControlParser.hpp"

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    
    auto node = std::make_shared<RemoteControlDataParser>("remote_control_node");
    
    // 启动Qt线程
    node->start();
    
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [node]() {
        rclcpp::spin_some(node);
    });
    timer.start(10); // 10ms 执行一次 spin_some
    
    // 运行 Qt 事件循环（主线程）
    int result = app.exec();
    
    // 清理
    rclcpp::shutdown();
    return result;
}