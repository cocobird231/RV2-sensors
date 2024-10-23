#pragma once
#include <thread>
#include <mutex>
#include <atomic>

#include <rcl/rcl.h>
#include <rosidl_runtime_c/string_functions.h>

#include <sensor_msgs/msg/nav_sat_fix.h>
// Issue: The status of NavSatStatus conflicts with the status defined in gps_fix_t.
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <sensor_msgs/msg/nav_sat_status.hpp>

#include <rv2_interfaces/timer.h>
#include <rv2_interfaces/rv2_interfaces.h>

struct SimpleGnssStatus
{
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    int8_t status = -1;
};

class GNSSNode : public rv2_interfaces::VehicleServiceNode
{
private:
    std::string mTopicName_ = "gnss_default";

    rcl_publisher_t mPub_;
    rcl_node_t mPubNode_;
    SimpleGnssStatus mMsg_;
    std::mutex mMsgMtx_;

    std::unique_ptr<rv2_interfaces::LiteTimer> mPubTm_;

private:
    void _pubTmCb();

public:
    GNSSNode(const std::shared_ptr<const rv2_interfaces::GenericParams> gParams);

    ~GNSSNode();

    void updateMsg(const SimpleGnssStatus& msg);
};
