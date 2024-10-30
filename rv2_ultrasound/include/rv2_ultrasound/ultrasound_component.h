#pragma once
#include <vector>
#include <array>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <rv2_interfaces/rv2_interfaces.h>
// #include <rv2_ultrasound/config.h>
#include "config.h"// TEST
// #include <rv2_ultrasound/serial_module.h>
#include "serial_module.h"// TEST

#define DEFAULT_ULTRASOUND_NODENAME "rv2_ultrasound_default_node"
#define DEFAULT_ULTRASOUND_TOPIC "ultrasound_default"

namespace rv2_sensors
{

class UltrasoundComponent : public rv2_interfaces::VehicleServiceNode
{
private:
    std::string mTopicName_ = DEFAULT_ULTRASOUND_TOPIC;
    std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, ULTRASOUND_MODULE_SIZE> mPubs_;

    std::array<sensor_msgs::msg::Range, ULTRASOUND_MODULE_SIZE> mMsgs_;

    std::string mDevicePath_ = "/dev/ttyACM0";
    int mDeviceBaud_ = 9600;
    std::unique_ptr<SerialModule> mSerialModule_;
    rv2_interfaces::unique_thread mSerialModuleTh_;

    std::atomic<bool> mExitF_;

public:
    UltrasoundComponent(const rclcpp::NodeOptions & options);

    ~UltrasoundComponent();

private:
    void _pubMsg(const std::array<float, ULTRASOUND_MODULE_SIZE>& range);

    void _grabSerial(double period_ms);

    bool isExit() const;
};

}
