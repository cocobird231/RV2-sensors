#pragma once
#include <thread>
#include <mutex>
#include <atomic>

#include <libgpsmm.h>

#include <rcl/rcl.h>
#include <rosidl_runtime_c/string_functions.h>

#include <sensor_msgs/msg/nav_sat_fix.h>
// Issue: The status of NavSatStatus conflicts with the status defined in gps_fix_t.
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <sensor_msgs/msg/nav_sat_status.hpp>

#include <rv2_interfaces/timer.h>
#include <rv2_interfaces/rv2_interfaces.h>

#define DEFAULT_GNSS_NODENAME "rv2_gnss_default_node"
#define DEFAULT_GNSS_TOPIC "gnss_default"

// Defined in gps.h
#define FIX_MODE_STR_NUM 4
static char *fix_mode_str[FIX_MODE_STR_NUM] = {
    "n/a",
    "None",
    "2D",
    "3D"
};

/**
GPS Quality indicator:
0: Fix not valid
1: GPS fix
2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
3: Not applicable
4: RTK Fixed, xFill
5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
6: INS Dead reckoning
*/

/**
gpsd fix status (not to be confused with GPS quality!):

#define STATUS_NO_FIX   0       // no, or unknown
#define STATUS_FIX      1       // yes, plain GPS (SPS Mode), without DGPS, PPS, RTK, DR, etc.
#define STATUS_DGPS_FIX 2       // yes, with DGPS
#define STATUS_RTK_FIX  3       // yes, with RTK Fixed
#define STATUS_RTK_FLT  4       // yes, with RTK Float
#define STATUS_DR       5       // yes, with dead reckoning
#define STATUS_GNSSDR   6       // yes, with GNSS + dead reckoning
#define STATUS_TIME     7       // yes, time only (surveyed in, manual)
#define STATUS_SIM      8       // yes, simulated
                                // Note that STATUS_SIM and MODE_NO_FIX can go together.
#define STATUS_PPS_FIX  9       // yes, Precise Positioning Service (PPS)
                                // Not to be confused with Pulse per Second (PPS)
                                // PPS is the encrypted military P(Y)-code
*/
#define FIX_STATUS_STR_NUM 10
static char *fix_status_str[FIX_STATUS_STR_NUM] = {
    "No Fix", 
    "Fix", 
    "DGPS", 
    "RTK", 
    "RTK Float", 
    "DR", 
    "GNSS + DR", 
    "Time Only", 
    "Simulated", 
    "PPS"
};

namespace rv2_sensors
{

struct SimpleGnssStatus
{
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    int8_t status = -1;
};

class GNSSComponent : public rv2_interfaces::VehicleServiceNode
{
private:
    std::string mTopicName_ = DEFAULT_GNSS_TOPIC;
    rcl_publisher_t mPub_;

    rv2_interfaces::unique_thread mGrabTh_;
    std::atomic<bool> mExitF_;

public:
    GNSSComponent(const rclcpp::NodeOptions & options);

    ~GNSSComponent();

private:
    void _publishMsg(const SimpleGnssStatus& msg);

    void _grabGNSSData(std::string host, int port, double period_ms);

public:
    bool isExit() const;
};

SimpleGnssStatus CvtGpsDataToSimpleGnssStatus(gps_data_t* gpsData);

}
