#include <rv2_gnss/gnss_node.h>

void GNSSNode::_pubTmCb()
{
    SimpleGnssStatus tmp;
    {
        std::lock_guard<std::mutex> msgLock(mMsgMtx_);
        tmp = mMsg_;
    }

    sensor_msgs__msg__NavSatFix *pubMsg = sensor_msgs__msg__NavSatFix__create();
    const auto& now_nsec = this->getTimestamp().nanoseconds();
    const auto& nowT = rmw_time_from_nsec(now_nsec);
    pubMsg->header.stamp.sec = static_cast<int32_t>(nowT.sec);
    pubMsg->header.stamp.nanosec = static_cast<uint32_t>(nowT.nsec);
    rosidl_runtime_c__String__assign(&pubMsg->header.frame_id, (mTopicName_ + "_link").c_str());
    pubMsg->latitude = tmp.latitude;
    pubMsg->longitude = tmp.longitude;
    pubMsg->altitude = tmp.altitude;
    /**
     * The statue we use here is the status of the GPS fix, which is defined in gps_fix_t.
     * There is an additional status -1 represents the status is unknown.
     */
    pubMsg->status.status = tmp.status;

    rcl_ret_t rc;
    rc = rcl_publish(&mPub_, pubMsg, NULL);
    if (rc != RCL_RET_OK)
    {
        // RCL_RET_PUBLISHER_INVALID is returned initially and then the message gets published
        RCLCPP_INFO(this->get_logger(), "rcl_publish error: %d\n", (int)rc);
        this->updateProcedureMonitor("_pubTmCb", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error in rcl_publish");
        return;
    }

    // Release ROS message.
    rosidl_runtime_c__String__fini(&pubMsg->header.frame_id);
    sensor_msgs__msg__NavSatFix__destroy(pubMsg);
    // RCLCPP_INFO(this->get_logger(), "[GNSSNode::_pubTmCb] Publish: lat: %3.3lf, lon: %3.3lf, alt: %3.3lf, status: %d\n", tmp.latitude, tmp.longitude, tmp.altitude, tmp.status);

    this->updateProcedureMonitor("_pubTmCb", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
    return;
}


GNSSNode::GNSSNode(const std::shared_ptr<const rv2_interfaces::GenericParams> gParams) : 
    rv2_interfaces::VehicleServiceNode(gParams), 
    rclcpp::Node(gParams->nodeName)
{
    // Get parameters
    double publishPeriod_ms = 500.0;
    double gpsModuleProcTimeout_ms = 6000.0;
    double msgPublishProcTimeout_ms = 1000.0;
    double msgUpdateProcTimeout_ms = 6000.0;
    this->getParam("topicName", mTopicName_, mTopicName_, "topicName: ", false);
    this->getParam("publishPeriod_ms", publishPeriod_ms, publishPeriod_ms, "publishPeriod_ms: ", false);
    this->getParam("gpsModuleProcTimeout_ms", gpsModuleProcTimeout_ms, gpsModuleProcTimeout_ms, "gpsModuleProcTimeout_ms: ", false);
    this->getParam("msgPublishProcTimeout_ms", msgPublishProcTimeout_ms, msgPublishProcTimeout_ms, "msgPublishProcTimeout_ms: ", false);
    this->getParam("msgUpdateProcTimeout_ms", msgUpdateProcTimeout_ms, msgUpdateProcTimeout_ms, "msgUpdateProcTimeout_ms: ", false);

    // Add procedure monitor
    this->addProcedureMonitor("gpsModule", std::chrono::nanoseconds((int64_t)(gpsModuleProcTimeout_ms * 1e6)));
    this->addProcedureMonitor("_pubTmCb", std::chrono::nanoseconds((int64_t)(msgPublishProcTimeout_ms * 1e6)));
    this->addProcedureMonitor("updateMsg", std::chrono::nanoseconds((int64_t)(msgUpdateProcTimeout_ms * 1e6)));

    // Initialize rcl content
    rcl_ret_t rc;
    static rcl_context_t context = rcl_get_zero_initialized_context();
    static rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    static rcl_allocator_t allocator = rcl_get_default_allocator();

    // Create init_options
    rc = rcl_init_options_init(&init_options, allocator);
    if (rc != RCL_RET_OK)
    {
        RCLCPP_INFO(this->get_logger(), "rcl_init_options_init error: %d\n", (int)rc);
        this->updateProcedureMonitor("_pubTmCb", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error rcl_init_options_init");
        return;
    }

    // Create context
    rc = rcl_init(0, nullptr, &init_options, &context);
    if (rc != RCL_RET_OK)
    {
        RCLCPP_INFO(this->get_logger(), "rcl_init error: %d\n", (int)rc);
        this->updateProcedureMonitor("_pubTmCb", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error in rcl_init");
        return;
    }

    // Initialize rcl node
    mPubNode_ = rcl_get_zero_initialized_node();
    static rcl_node_options_t node_ops = rcl_node_get_default_options();
    rc = rcl_node_init(&mPubNode_, (gParams->nodeName + "_pubnode").c_str(), this->get_namespace(), &context, &node_ops);
    if (rc != RCL_RET_OK)
    {
        RCLCPP_INFO(this->get_logger(), "rcl_node_init init error: %d\n", (int)rc);
        this->updateProcedureMonitor("_pubTmCb", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error in rcl_node_init");
        return;
    }

    // Initialize rcl publisher
    const static rosidl_message_type_support_t *my_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix);
    mPub_ = rcl_get_zero_initialized_publisher();
    static rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
    rc = rcl_publisher_init(&mPub_, &mPubNode_, my_type_support, mTopicName_.c_str(), &pub_options);
    if (RCL_RET_OK != rc)
    {
        RCLCPP_INFO(this->get_logger(), "rcl_publisher_init error: %d.\n", (int)rc);
        this->updateProcedureMonitor("_pubTmCb", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_ERROR, "Error in rcl_publisher_init");
        return;
    }

    mPubTm_ = std::make_unique<rv2_interfaces::LiteTimer>(publishPeriod_ms, std::bind(&GNSSNode::_pubTmCb, this));
    mPubTm_->start();

    RCLCPP_INFO(this->get_logger(), "[GNSSNode] Constructed.");
}

GNSSNode::~GNSSNode()
{
    mPubTm_->stop();
    mPubTm_.reset();

    rcl_publisher_fini(&mPub_, &mPubNode_);
    rcl_node_fini(&mPubNode_);
    RCLCPP_INFO(this->get_logger(), "[GNSSNode] Destructed.");
}

void GNSSNode::updateMsg(const SimpleGnssStatus& msg)
{
    std::lock_guard<std::mutex> msgLock(mMsgMtx_);
    mMsg_ = msg;
    this->updateProcedureMonitor("updateMsg", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
}
