#include "header.h"

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <atomic>

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
    std::shared_ptr<const Params> params;
    {
        auto tmpNode = vehicle_interfaces::GenTmpNode<Params>("tmp_ultrasound_params_");
        params = std::make_shared<const Params>(tmpNode->nodeName + "_params_node");
    }
    std::chrono::nanoseconds loopDelay_ns((int64_t)(params->publishInterval_s * 1e9 / 2));

    auto pubNode = std::make_shared<UltrasoundPublisher>(params);
    pubNode->addProcedureMonitor("SerialModule", std::chrono::nanoseconds((int64_t)(params->serialModuleProcTimeout_ms * 1e6)));
    auto execPubNode = vehicle_interfaces::ExecNode(pubNode);
    execPubNode.start();

    // Serial module
    auto serialModule = std::make_unique<SerialModule>(params->devicePath, params->deviceBaud);

    while (!__globalExitFlag.load())
    {
        std::this_thread::sleep_for(loopDelay_ns);
        if (!serialModule || serialModule->isExit())
        {
            pubNode->updateProcedureMonitor("SerialModule", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "SerialModule is not responding");
            serialModule.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            serialModule = std::make_unique<SerialModule>(params->devicePath, params->deviceBaud);
            continue;
        }

        std::array<float, ULTRASOUND_MODULE_SIZE> msg;
        std::chrono::system_clock::time_point msgTs;
        if (serialModule->getMsg(msg, msgTs))
        {
            if (std::chrono::system_clock::now() - msgTs > std::chrono::seconds(1))
            {
                pubNode->updateProcedureMonitor("SerialModule", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Ultrasound module is not responding");
                std::cerr << "Warning: Ultrasound module is not responding" << std::endl;
                serialModule.reset();
                continue;
            }
            pubNode->updateMsg(msg);
            pubNode->updateProcedureMonitor("SerialModule", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
        }
    }

    execPubNode.stop();
    rclcpp::shutdown();
    return 0;
}
