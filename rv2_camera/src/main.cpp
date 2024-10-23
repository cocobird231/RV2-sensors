#include <rv2_camera/camera_node.h>

static std::atomic<bool> __globalExitFlag(false);

int main(int argc, char** argv)
{
    // Ctrl-c handler
    signal(SIGINT, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    // SIGTERM handler
    signal(SIGTERM, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    // SIGKILL handler
    signal(SIGKILL, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    rclcpp::init(argc, argv);
    std::shared_ptr<rv2_interfaces::GenericParams> gParams;
    {
        auto tmpNode = rv2_interfaces::GenTmpNode<rv2_interfaces::GenericParams>("tmp_rv2_camera_params_");
        gParams = std::make_shared<rv2_interfaces::GenericParams>(tmpNode->nodeName + "_" + tmpNode->id + "_params_node");
    }
    gParams->nodeName += "_" + gParams->id;

    auto cameraNode = std::make_shared<CameraNode>(gParams);
    auto execNode = rv2_interfaces::ExecNode(cameraNode);
    execNode.start();

    while (!__globalExitFlag.load() && !cameraNode->isExit())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    execNode.stop();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
