#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <libgpsmm.h>
#include <cmath>
#include <atomic>

#include <rv2_gnss/gnss_node.h>

static std::atomic<bool> __globalExitFlag(false);

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


// Convert the GPS data to a ROS2 NavSatFix message
SimpleGnssStatus CvtGpsDataToSimpleGnssStatus(gps_data_t* gpsData)
{
    SimpleGnssStatus ret;
    ret.latitude = gpsData->fix.latitude;
    ret.longitude = gpsData->fix.longitude;
    ret.altitude = gpsData->fix.altitude;
    switch (gpsData->fix.status)
    {
    case STATUS_NO_FIX:
        ret.status = sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
        break;
    case STATUS_FIX:
        ret.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
        break;
    case STATUS_DGPS_FIX:
        ret.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
        break;
    case STATUS_RTK_FIX:
        ret.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
        break;
    case STATUS_RTK_FLT:
        ret.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
        break;
    default:
        ret.status = sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
        break;
    }
    // ret.status = gpsData->fix.status;
    return ret;
}


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

    // ROS2 init
    rclcpp::init(argc, argv);

    std::shared_ptr<rv2_interfaces::GenericParams> gParams;
    {
        auto tmpNode = rv2_interfaces::GenTmpNode<rv2_interfaces::GenericParams>("tmp_rv2_gnss_params_");
        gParams = std::make_shared<rv2_interfaces::GenericParams>(tmpNode->nodeName + "_" + tmpNode->id + "_params_node");
    }
    gParams->nodeName += "_" + gParams->id;

    auto gnssNode = std::make_shared<GNSSNode>(gParams);
    auto execNode = rv2_interfaces::ExecNode(gnssNode);
    execNode.start();

    // Open the GPS device
    struct gps_data_t gps_data;

    if (gps_open("localhost", "2947", &gps_data) != 0)
    {
        std::cerr << "Error: Unable to open the GPS device. Exit program in 1s..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return 1;
    }

    gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

    // Main loop
    while (!__globalExitFlag.load())
    {
        if (!gps_waiting(&gps_data, 5000000))
        {
            std::cerr << "Error: Timeout waiting for data from the GPS device" << std::endl;
            gnssNode->updateProcedureMonitor("gpsModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Timeout waiting for data from the GPS device");
            continue;
        }

        if (gps_read(&gps_data, NULL, 0) == -1)
        {
            std::cerr << "Error: Unable to read data from the GPS device" << std::endl;
            gnssNode->updateProcedureMonitor("gpsModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Unable to read data from the GPS device");
            continue;
        }

        if ((MODE_SET & gps_data.set) != MODE_SET)
        {
            std::cerr << "Error: Unable to set the GPS mode" << std::endl;
            gnssNode->updateProcedureMonitor("gpsModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Unable to set the GPS mode");
            continue;
        }

        if (gps_data.fix.mode < 0 || gps_data.fix.mode >= FIX_MODE_STR_NUM || gps_data.fix.status < 0 || gps_data.fix.status >= FIX_STATUS_STR_NUM)
        {
            gps_data.fix.mode = MODE_NOT_SEEN;
            gps_data.fix.status = STATUS_NO_FIX;
        }

        // printf("Fix mode: %s (%s) ", fix_mode_str[gps_data.fix.mode], fix_status_str[gps_data.fix.status]);

        if ((TIME_SET & gps_data.set) == TIME_SET)
        {
            // printf("Time: %ld.%09ld ", gps_data.fix.time.tv_sec, gps_data.fix.time.tv_nsec);
        }
        else
        {
            // printf("n/a ");
        }

        if (std::isfinite(gps_data.fix.latitude) && std::isfinite(gps_data.fix.longitude))
        {
            // printf("Lat: %.6f, Lon: %.6f\n", gps_data.fix.latitude, gps_data.fix.longitude);
            gnssNode->updateMsg(CvtGpsDataToSimpleGnssStatus(&gps_data));
        }
        else
        {
            // printf("Lat: n/a, Lon: n/a\n");
        }
        gnssNode->updateProcedureMonitor("gpsModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
    }

    // Clean up gps_data
    gps_stream(&gps_data, WATCH_DISABLE, NULL);
    gps_close(&gps_data);

    // ROS2 shutdown
    execNode.stop();
    rclcpp::shutdown();
    return 0;
}
