#pragma once
#include <thread>
#include <mutex>
#include <atomic>

#include "rcl/rcl.h"
#include <sensor_msgs/msg/nav_sat_fix.h>

// Issue: The status of NavSatStatus conflicts with the status defined in gps_fix_t.
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <sensor_msgs/msg/nav_sat_status.hpp>

#include "vehicle_interfaces/timer.h"
#include "vehicle_interfaces/vehicle_interfaces.h"


struct SimpleGnssStatus
{
    double latitude;
    double longitude;
    double altitude;
    int8_t status;
};

class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string topicName = "gnss_0";
    float publishInterval_s = 0.1;
    std::string gpsDevice = "/dev/ttyUSB0";
    int gpsBaud_dec = 115200;
    std::string ntripCaster = "61.220.23.236";
    std::string ntripPort = "2101";
    std::string ntripMountpoint = "/MIRDC01";
    std::string ntripUsername = "";
    std::string ntripPassword = "";
    float gpsModuleProcTimeout_ms = 6000;
    float msgUpdateProcTimeout_ms = 6000;
    float msgPublishProcTimeout_ms = 6000;

private:
    void _getParams()
    {
        this->get_parameter("topicName", this->topicName);
        this->get_parameter("publishInterval_s", this->publishInterval_s);
        this->get_parameter("gpsDevice", this->gpsDevice);
        this->get_parameter("gpsBaud_dec", this->gpsBaud_dec);
        this->get_parameter("ntripCaster", this->ntripCaster);
        this->get_parameter("ntripPort", this->ntripPort);
        this->get_parameter("ntripMountpoint", this->ntripMountpoint);
        this->get_parameter("ntripUsername", this->ntripUsername);
        this->get_parameter("ntripPassword", this->ntripPassword);
        this->get_parameter("gpsModuleProcTimeout_ms", this->gpsModuleProcTimeout_ms);
        this->get_parameter("msgUpdateProcTimeout_ms", this->msgUpdateProcTimeout_ms);
        this->get_parameter("msgPublishProcTimeout_ms", this->msgPublishProcTimeout_ms);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("topicName", this->topicName);
        this->declare_parameter<float>("publishInterval_s", this->publishInterval_s);
        this->declare_parameter<std::string>("gpsDevice", this->gpsDevice);
        this->declare_parameter<int>("gpsBaud_dec", this->gpsBaud_dec);
        this->declare_parameter<std::string>("ntripCaster", this->ntripCaster);
        this->declare_parameter<std::string>("ntripPort", this->ntripPort);
        this->declare_parameter<std::string>("ntripMountpoint", this->ntripMountpoint);
        this->declare_parameter<std::string>("ntripUsername", this->ntripUsername);
        this->declare_parameter<std::string>("ntripPassword", this->ntripPassword);
        this->declare_parameter<float>("gpsModuleProcTimeout_ms", this->gpsModuleProcTimeout_ms);
        this->declare_parameter<float>("msgUpdateProcTimeout_ms", this->msgUpdateProcTimeout_ms);
        this->declare_parameter<float>("msgPublishProcTimeout_ms", this->msgPublishProcTimeout_ms);

        this->_getParams();
    }
};


class GNSSPubNode : public vehicle_interfaces::VehicleServiceNode
{
private:
    const std::shared_ptr<const Params> params_;

    rcl_publisher_t pub_;
    std::atomic<bool> pubF_;
    SimpleGnssStatus msg_;
    std::mutex msgMtx_;

    std::unique_ptr<vehicle_interfaces::LiteTimer> pubTm_;

private:
    void _pubTmCb()
    {
        if(!pubF_)
        {
            rcl_context_t context = rcl_get_zero_initialized_context();
            rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
            rcl_allocator_t allocator = rcl_get_default_allocator();
            rcl_ret_t rc;

            // create init_options
            rc = rcl_init_options_init(&init_options, allocator);
            if (rc != RCL_RET_OK)
            {
                printf("Error rcl_init_options_init.\n");
                this->updateProcedureMonitor("_pubTmCb", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error rcl_init_options_init");
                return;
            }

            // create context
            rc = rcl_init(0, nullptr, &init_options, &context);
            if (rc != RCL_RET_OK)
            {
                printf("Error in rcl_init.\n");
                this->updateProcedureMonitor("_pubTmCb", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error in rcl_init");
                return;
            }

            std::string namespace_(this->get_namespace());
            std::string topic_name(this->params_->topicName);

            // create rcl_node
            rcl_node_t my_node = rcl_get_zero_initialized_node();
            rcl_node_options_t node_ops = rcl_node_get_default_options();
            rc = rcl_node_init(&my_node, (const char *)(this->params_->nodeName + "_pub_node").c_str(), namespace_.c_str(), &context, &node_ops);
            if (rc != RCL_RET_OK)
            {
                printf("Error in rcl_node_init\n");
                this->updateProcedureMonitor("_pubTmCb", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error in rcl_node_init");
                return;
            }

            const rosidl_message_type_support_t * my_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix);

            rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();

            // Initialize Publisher
            rc = rcl_publisher_init(
                &this->pub_,
                &my_node,
                my_type_support,
                topic_name.c_str(),
                &pub_options);
            if (RCL_RET_OK != rc)
            {
                printf("Error in rcl_publisher_init %s.\n", topic_name.c_str());
                this->updateProcedureMonitor("_pubTmCb", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error in rcl_publisher_init");
                return;
            }
            // Node, publisher was successfully created
            this->pubF_ = true;

            return;
        }

        SimpleGnssStatus tmp;
        {
            std::lock_guard<std::mutex> msgLock(this->msgMtx_);
            tmp = this->msg_;
        }

        sensor_msgs__msg__NavSatFix *pubMsg = sensor_msgs__msg__NavSatFix__create();
        const auto& now_nsec = this->getTimestamp().nanoseconds();
        const auto& nowT = rmw_time_from_nsec(now_nsec);
        pubMsg->header.stamp.sec = static_cast<int32_t>(nowT.sec);
        pubMsg->header.stamp.nanosec = static_cast<uint32_t>(nowT.nsec);
        pubMsg->latitude = tmp.latitude;
        pubMsg->longitude = tmp.longitude;
        pubMsg->altitude = tmp.altitude;
        /**
         * The statue we use here is the status of the GPS fix, which is defined in gps_fix_t.
         * There is an additional status -1 represents the status is unknown.
         */
        pubMsg->status.status = tmp.status;

        rcl_ret_t rc;
        rc = rcl_publish(&this->pub_, pubMsg, NULL);
        if (rc != RCL_RET_OK)
        {
            // RCL_RET_PUBLISHER_INVALID is returned initially and then the message gets published
            this->updateProcedureMonitor("_pubTmCb", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error in rcl_publish");
            return;
        }

        // Release ROS message.
        sensor_msgs__msg__NavSatFix__destroy(pubMsg);
        // printf("[GNSSPubNode::_pubTmCb] Publish: lat: %3.3lf, lon: %3.3lf, alt: %3.3lf, status: %d\n", tmp.latitude, tmp.longitude, tmp.altitude, tmp.status);

        this->updateProcedureMonitor("_pubTmCb", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
        return;
    }

public:
    GNSSPubNode(const std::shared_ptr<const Params> params) : 
        vehicle_interfaces::VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        params_(params)
    {
        this->addProcedureMonitor("_pubTmCb", std::chrono::nanoseconds((int64_t)(params->msgPublishProcTimeout_ms * 1e6)));
        this->addProcedureMonitor("updateMsg", std::chrono::nanoseconds((int64_t)(params->msgUpdateProcTimeout_ms * 1e6)));

        this->pubTm_ = std::make_unique<vehicle_interfaces::LiteTimer>(params->publishInterval_s * 1000, std::bind(&GNSSPubNode::_pubTmCb, this));
        this->pubTm_->start();
    }

    void updateMsg(const SimpleGnssStatus& msg)
    {
        std::lock_guard<std::mutex> msgLock(this->msgMtx_);
        this->msg_ = msg;
        this->updateProcedureMonitor("updateMsg", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
    }
};
