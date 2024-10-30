// #include <rv2_ultrasound/ultrasound_component.h>
#include "../include/rv2_ultrasound/ultrasound_component.h"// TEST

namespace rv2_sensors
{

UltrasoundComponent::UltrasoundComponent(const rclcpp::NodeOptions & options) : 
    rv2_interfaces::VehicleServiceNode(DEFAULT_ULTRASOUND_NODENAME, options), 
    rclcpp::Node(DEFAULT_ULTRASOUND_NODENAME, options), 
    mExitF_(false)
{
    // Get parameters
    std::vector<double> topicIds = { 0, 1, 2, 3 };
    double publishPeriod_ms = 100.0;
    GetParamRawPtr(this, "topicName", mTopicName_, mTopicName_, "topicName: ", false);
    GetParamRawPtr(this, "topicIds", topicIds, topicIds, "topicIds: ", false);
    GetParamRawPtr(this, "publishPeriod_ms", publishPeriod_ms, publishPeriod_ms, "publishPeriod_ms: ", false);

    // Serial module parameters
    double minRange = 0.2;
    double maxRange = 8.0;
    double fov = 15.0;
    GetParamRawPtr(this, "devicePath", mDevicePath_, mDevicePath_, "devicePath: ", false);
    GetParamRawPtr(this, "deviceBaud", mDeviceBaud_, mDeviceBaud_, "deviceBaud: ", false);
    GetParamRawPtr(this, "minRange", minRange, minRange, "minRange: ", false);
    GetParamRawPtr(this, "maxRange", maxRange, maxRange, "maxRange: ", false);
    GetParamRawPtr(this, "fov", fov, fov, "fov: ", false);

    double serialModuleProcTimeout_ms = 100.0;
    double msgPublishProcTimeout_ms = 200.0;
    GetParamRawPtr(this, "serialModuleProcTimeout_ms", serialModuleProcTimeout_ms, serialModuleProcTimeout_ms, "serialModuleProcTimeout_ms: ", false);
    GetParamRawPtr(this, "msgPublishProcTimeout_ms", msgPublishProcTimeout_ms, msgPublishProcTimeout_ms, "msgPublishProcTimeout_ms: ", false);

    // Add procedure monitor
    this->addProcedureMonitor("SerialModule", std::chrono::nanoseconds((int64_t)(serialModuleProcTimeout_ms * 1e6)));
    this->addProcedureMonitor("_pubMsg", std::chrono::nanoseconds((int64_t)(msgPublishProcTimeout_ms * 1e6)));

    // Init msgs and publishers.
    std::vector<int> ids = { 0, 1, 2, 3 };
    if (ULTRASOUND_MODULE_SIZE != topicIds.size())
    {
        RCLCPP_ERROR(this->get_logger(), "topicIds size must be %d", ULTRASOUND_MODULE_SIZE);
        RCLCPP_ERROR(this->get_logger(), "Use default topicIds...");
    }
    else
    {
        for (int i = 0; i < ULTRASOUND_MODULE_SIZE; i++)
            ids[i] = static_cast<int>(topicIds[i]);
    }

    for (int i = 0; i < ULTRASOUND_MODULE_SIZE; i++)
    {
        std::string topicName = mTopicName_ + "_" + std::to_string(ids[i]);
        mMsgs_[i] = sensor_msgs::msg::Range();
        mMsgs_[i].header.frame_id = topicName + "_link";
        mMsgs_[i].radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        mMsgs_[i].field_of_view  = DEG2RAD(fov);// ROS2 FoV in rad
        mMsgs_[i].min_range = minRange;
        mMsgs_[i].max_range = maxRange;
        mMsgs_[i].range = 0.0;

        mPubs_[i] = this->create_publisher<sensor_msgs::msg::Range>(topicName, 10);
    }

    // Serial module
    mSerialModuleTh_ = rv2_interfaces::make_unique_thread(&UltrasoundComponent::_grabSerial, this, publishPeriod_ms / 2);

    RCLCPP_INFO(this->get_logger(), "[UltrasoundComponent] Constructed.");
}



UltrasoundComponent::~UltrasoundComponent()
{
    mExitF_.store(true);
    mSerialModuleTh_.reset();
}



void UltrasoundComponent::_pubMsg(const std::array<float, ULTRASOUND_MODULE_SIZE>& range)
{
    if (mExitF_.load())
    {
        RCLCPP_ERROR(this->get_logger(), "[UltrasoundComponent::_pubMsg] Exit flag is set. Exit publishing ultrasound message.");
        return;
    }

    for (int i = 0; i < ULTRASOUND_MODULE_SIZE; i++)
    {
        auto msg = std::make_unique<sensor_msgs::msg::Range>();
        *msg.get() = mMsgs_[i];
        msg->header.stamp = this->get_clock()->now();
        msg->range = range[i];
        mPubs_[i]->publish(std::move(msg));
    }

    this->updateProcedureMonitor("_pubMsg", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
}



void UltrasoundComponent::_grabSerial(double period_ms)
{
    RCLCPP_INFO(this->get_logger(), "[UltrasoundComponent::_grabSerial] Start grabbing serial data.");
    auto loopDelay = std::chrono::milliseconds((int64_t)period_ms);
    while (!mExitF_.load())
    {
        std::this_thread::sleep_for(loopDelay);
        if (!mSerialModule_ || mSerialModule_->isExit())
        {
            this->updateProcedureMonitor("SerialModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "SerialModule is not responding");
            RCLCPP_ERROR(this->get_logger(), "[UltrasoundComponent::_grabSerial] SerialModule is not responding.");
            mSerialModule_.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            mSerialModule_ = std::make_unique<SerialModule>(mDevicePath_, mDeviceBaud_);
            continue;
        }

        std::array<float, ULTRASOUND_MODULE_SIZE> msg;
        std::chrono::system_clock::time_point msgTs;
        if (mSerialModule_->getMsg(msg, msgTs))
        {
            if (std::chrono::system_clock::now() - msgTs > std::chrono::seconds(1))
            {
                this->updateProcedureMonitor("SerialModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Ultrasound module is not responding");
                RCLCPP_ERROR(this->get_logger(), "[UltrasoundComponent::_grabSerial] Ultrasound module is not responding.");
                mSerialModule_.reset();
                continue;
            }
            this->_pubMsg(msg);
            this->updateProcedureMonitor("SerialModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
        }
        else
        {
            this->updateProcedureMonitor("SerialModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Failed to get ultrasound data");
            RCLCPP_ERROR(this->get_logger(), "[UltrasoundComponent::_grabSerial] Failed to get ultrasound data.");
        }
    }
    RCLCPP_INFO(this->get_logger(), "[UltrasoundComponent::_grabSerial] Exit grabbing serial data.");
}



bool UltrasoundComponent::isExit() const
{
    return mExitF_.load();
}

}



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rv2_sensors::UltrasoundComponent)
