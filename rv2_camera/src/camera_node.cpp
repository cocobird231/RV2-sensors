// #include <rv2_camera/camera_component.h>
#include "../include/rv2_camera/camera_component.h"// TEST

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    options.use_intra_process_comms(true);
    auto node = std::make_shared<rv2_sensors::CameraComponent>(options);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
