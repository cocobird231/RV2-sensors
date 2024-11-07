// Copyright 2022 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>

#include "zed_components/zed_camera_component.hpp"
#include "rv2_interfaces/rv2_interfaces.h"
#include "rv2_interfaces/params.h"

static std::atomic<bool> __globalExitFlag(false);

class ZEDNode : public stereolabs::ZedCamera, public rv2_interfaces::BaseDevManageNode
{
private:
    std::atomic<bool> mExitF_;
public:
    ZEDNode(const rclcpp::NodeOptions& options) : 
        stereolabs::ZedCamera(options), 
        rv2_interfaces::BaseDevManageNode(this), 
        mExitF_(false)
    {
        this->registerForceExitCallback(std::bind(&ZEDNode::_forceExit, this, std::placeholders::_1));
    }

    ~ZEDNode()
    {
        mExitF_.store(true);
    }

private:
    void _forceExit(const rv2_interfaces::DevManageForceExit& msg)
    {
        mExitF_.store(true);
        __globalExitFlag.store(true);
    }

    bool isExit() const
    {
        return mExitF_.load();
    }
};

int main(int argc, char * argv[])
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

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    auto zedCamNode = std::make_shared<ZEDNode>(options);
    zedCamNode->addProcedureMonitor("zedMainTh", std::chrono::nanoseconds(1000000000));
    auto zedCamExecNode = std::make_shared<rv2_interfaces::ExecNode>(zedCamNode);
    zedCamExecNode->start();

    while (!__globalExitFlag.load())
    {
        zedCamNode->updateProcedureMonitor("zedMainTh", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    zedCamExecNode.reset();
    zedCamNode.reset();
    // set SIGINT
    raise(SIGINT);
    rclcpp::shutdown();
    raise(SIGINT);

    return 0;
}
