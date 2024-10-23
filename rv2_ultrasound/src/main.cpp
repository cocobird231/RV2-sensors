#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <atomic>

#include <rv2_ultrasound/ultrasound_node.h>

static std::atomic<bool> __globalExitFlag(false);

int main(int argc, char **argv)
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
        auto tmpNode = rv2_interfaces::GenTmpNode<rv2_interfaces::GenericParams>("tmp_rv2_ultrasound_params_");
        gParams = std::make_shared<rv2_interfaces::GenericParams>(tmpNode->nodeName + "_" + tmpNode->id + "_params_node");
    }
    gParams->nodeName += "_" + gParams->id;

    auto ultrasoundNode = std::make_shared<UltrasoundNode>(gParams);
    auto execNode = rv2_interfaces::ExecNode(ultrasoundNode);
    execNode.start();

    while (!__globalExitFlag.load() && !ultrasoundNode->isExit())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    execNode.stop();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
