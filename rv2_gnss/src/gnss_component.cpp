#include "rv2_gnss/gnss_component.h"

namespace rv2_sensors
{

/**
 * ================================================================
 * GNSSComponent Constructor and Destructor
 * ================================================================
 */



GNSSComponent::GNSSComponent(const rclcpp::NodeOptions & options) : 
    rv2_interfaces::VehicleServiceNode(DEFAULT_GNSS_NODENAME, options), 
    rclcpp::Node(DEFAULT_GNSS_NODENAME, options), 
    mExitF_(false)
{
    // Get parameters
    GetParamRawPtr(this, "topicName", mTopicName_, mTopicName_, "topicName: ", false);

    std::string gpsdHost = "localhost";
    int gpsdPort = 2947;
    double publishPeriod_ms = 500.0;
    double gpsModuleProcTimeout_ms = 6000.0;
    double msgPublishProcTimeout_ms = 1000.0;
    GetParamRawPtr(this, "gpsdHost", gpsdHost, gpsdHost, "gpsdHost: ", false);
    GetParamRawPtr(this, "gpsdPort", gpsdPort, gpsdPort, "gpsdPort: ", false);
    GetParamRawPtr(this, "publishPeriod_ms", publishPeriod_ms, publishPeriod_ms, "publishPeriod_ms: ", false);
    GetParamRawPtr(this, "gpsModuleProcTimeout_ms", gpsModuleProcTimeout_ms, gpsModuleProcTimeout_ms, "gpsModuleProcTimeout_ms: ", false);
    GetParamRawPtr(this, "msgPublishProcTimeout_ms", msgPublishProcTimeout_ms, msgPublishProcTimeout_ms, "msgPublishProcTimeout_ms: ", false);

    // Add procedure monitor
    this->addProcedureMonitor("gpsModule", std::chrono::nanoseconds((int64_t)(gpsModuleProcTimeout_ms * 1e6)));
    this->addProcedureMonitor("_publishMsg", std::chrono::nanoseconds((int64_t)(msgPublishProcTimeout_ms * 1e6)));

    // Initialize rcl publisher
    rcl_ret_t rc;
    const static rosidl_message_type_support_t *my_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix);
    mPub_ = rcl_get_zero_initialized_publisher();
    static rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
    // rc = rcl_publisher_init(&mPub_, &mPubNode_, my_type_support, mTopicName_.c_str(), &pub_options);
    rc = rcl_publisher_init(&mPub_, this->get_node_base_interface()->get_rcl_node_handle(), my_type_support, mTopicName_.c_str(), &pub_options);
    if (RCL_RET_OK != rc)
    {
        RCLCPP_INFO(this->get_logger(), "rcl_publisher_init error: %d.\n", (int)rc);
        this->updateProcedureMonitor("_publishMsg", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error in rcl_publisher_init");
        return;
    }

    // Start grab thread
    mGrabTh_ = rv2_interfaces::make_unique_thread(&GNSSComponent::_grabGNSSData, this, gpsdHost, gpsdPort, publishPeriod_ms);

    RCLCPP_INFO(this->get_logger(), "[GNSSComponent] Constructed.");
}



GNSSComponent::~GNSSComponent()
{
    mExitF_.store(true);
    mGrabTh_.reset();
    rcl_publisher_fini(&mPub_, this->get_node_base_interface()->get_rcl_node_handle());
    // rcl_publisher_fini(&mPub_, &mPubNode_);
    // rcl_node_fini(&mPubNode_);
    RCLCPP_INFO(this->get_logger(), "[GNSSComponent] Destructed.");
}



/**
 * ================================================================
 * GNSSComponent Private Methods
 * ================================================================
 */



void GNSSComponent::_publishMsg(const SimpleGnssStatus& msg)
{
    if (mExitF_.load())
    {
        RCLCPP_WARN(this->get_logger(), "[GNSSComponent::_publishMsg] Exit flag is set. Stop publishing.");
        return;
    }

    sensor_msgs__msg__NavSatFix *pubMsg = sensor_msgs__msg__NavSatFix__create();
    const auto& now_nsec = this->get_clock()->now().nanoseconds();
    const auto& nowT = rmw_time_from_nsec(now_nsec);
    pubMsg->header.stamp.sec = static_cast<int32_t>(nowT.sec);
    pubMsg->header.stamp.nanosec = static_cast<uint32_t>(nowT.nsec);
    rosidl_runtime_c__String__assign(&pubMsg->header.frame_id, (mTopicName_ + "_link").c_str());
    pubMsg->latitude = msg.latitude;
    pubMsg->longitude = msg.longitude;
    pubMsg->altitude = msg.altitude;
    /**
     * The statue we use here is the status of the GPS fix, which is defined in gps_fix_t.
     * There is an additional status -1 represents the status is unknown.
     */
    pubMsg->status.status = msg.status;

    rcl_ret_t rc;
    rc = rcl_publish(&mPub_, pubMsg, NULL);
    if (rc != RCL_RET_OK)
    {
        // RCL_RET_PUBLISHER_INVALID is returned initially and then the message gets published
        RCLCPP_INFO(this->get_logger(), "rcl_publish error: %d\n", (int)rc);
        this->updateProcedureMonitor("_publishMsg", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error in rcl_publish");
    }
    else
        this->updateProcedureMonitor("_publishMsg", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);

    // Release ROS message.
    rosidl_runtime_c__String__fini(&pubMsg->header.frame_id);
    sensor_msgs__msg__NavSatFix__destroy(pubMsg);
    // RCLCPP_INFO(this->get_logger(), "[GNSSComponent::_publishMsg] Publish: lat: %3.3lf, lon: %3.3lf, alt: %3.3lf, status: %d\n", msg.latitude, msg.longitude, msg.altitude, msg.status);

    return;
}


void GNSSComponent::_grabGNSSData(std::string host, int port, double period_ms)
{
    // Open the GPS device
    struct gps_data_t gps_data;

    if (gps_open(host.c_str(), std::to_string(port).c_str(), &gps_data) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error: Unable to open the GPS device. Exit program in 1s...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        raise(SIGINT);
    }

    gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

    while (!mExitF_.load())
    {
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(period_ms));
        if (!gps_waiting(&gps_data, 5000000))
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Timeout waiting for data from the GPS device.");
            this->updateProcedureMonitor("gpsModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Timeout waiting for data from the GPS device");
            continue;
        }

        if (gps_read(&gps_data, NULL, 0) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Unable to read data from the GPS device.");
            this->updateProcedureMonitor("gpsModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Unable to read data from the GPS device");
            continue;
        }

        if ((MODE_SET & gps_data.set) != MODE_SET)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Unable to set the GPS mode.");
            this->updateProcedureMonitor("gpsModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Unable to set the GPS mode");
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
            this->_publishMsg(CvtGpsDataToSimpleGnssStatus(&gps_data));
        }
        else
        {
            // printf("Lat: n/a, Lon: n/a\n");
        }
        this->updateProcedureMonitor("gpsModule", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
    }

    // Clean up gps_data
    gps_stream(&gps_data, WATCH_DISABLE, NULL);
    gps_close(&gps_data);
}



/**
 * ================================================================
 * GNSSComponent Public Methods
 * ================================================================
 */



bool GNSSComponent::isExit() const
{
    return mExitF_.load();
}



/**
 * ================================================================
 * Functions
 * ================================================================
 */



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

}



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rv2_sensors::GNSSComponent)
