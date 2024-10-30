// #include <rv2_gnss/gnss_component.h>
#include "../include/rv2_gnss/gnss_component.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    options.use_intra_process_comms(true);
    auto node = std::make_shared<rv2_sensors::GNSSComponent>(options);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
