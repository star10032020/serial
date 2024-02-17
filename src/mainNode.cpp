#include "serial/listenNode.hpp"
#include "serial/talkerNode.hpp"
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::Node::SharedPtr talkernode = std::make_shared<talkerNode>();
    rclcpp::Node::SharedPtr listennode = std::make_shared<listenNode>();
    executor.add_node(talkernode);
    executor.add_node(listennode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}