#include "rv2_gnss/gnss_component.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    options.use_intra_process_comms(true);
    auto node = std::make_shared<rv2_sensors::GNSSComponent>(options);
    auto execNode = rv2_interfaces::ExecNode(node);
    execNode.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (!node->isExit())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    execNode.stop();
    rclcpp::shutdown();
    return 0;
}
