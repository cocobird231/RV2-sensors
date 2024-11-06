#include "rv2_camera/camera_component.h"

namespace rv2_sensors
{

/**
 * ================================================================
 * CameraComponent Constructor and Destructor
 * ================================================================
 */



CameraComponent::CameraComponent(const rclcpp::NodeOptions & options) : 
    rv2_interfaces::VehicleServiceNode(DEFAULT_CAMERA_NODENAME, options), 
    rclcpp::Node(DEFAULT_CAMERA_NODENAME, options), 
    mExitF_(false)
{
    // Get parameters
    GetParamRawPtr(this, "topicName", mTopicName_, mTopicName_, "topicName: ", false);
    GetParamRawPtr(this, "publishWidth", mPublishWidth_, mPublishWidth_, "publishWidth: ", false);
    GetParamRawPtr(this, "publishHeight", mPublishHeight_, mPublishHeight_, "publishHeight: ", false);
    GetParamRawPtr(this, "useCompression", mUseCompression_, mUseCompression_, "useCompression: ", false);
    GetParamRawPtr(this, "compressionQuality", mCompressionQuality_, mCompressionQuality_, "compressionQuality: ", false);

    bool cameraAutoSetting = true;
    GetParamRawPtr(this, "cameraCapID", mCameraCapID_, mCameraCapID_, "cameraCapID: ", false);
    GetParamRawPtr(this, "cameraAutoSetting", cameraAutoSetting, cameraAutoSetting, "cameraAutoSetting: ", false);
    GetParamRawPtr(this, "cameraFPS", mCameraFPS_, mCameraFPS_, "cameraFPS: ", false);
    GetParamRawPtr(this, "cameraWidth", mCameraWidth_, mCameraWidth_, "cameraWidth: ", false);
    GetParamRawPtr(this, "cameraHeight", mCameraHeight_, mCameraHeight_, "cameraHeight: ", false);
    GetParamRawPtr(this, "cameraUseColor", mCameraUseColor_, mCameraUseColor_, "cameraUseColor: ", false);

    double imagePublishProcTimeout_ms = 200.0;
    double caminfoPublishProcTimeout_ms = 2000.0;
    GetParamRawPtr(this, "imagePublishProcTimeout_ms", imagePublishProcTimeout_ms, imagePublishProcTimeout_ms, "imagePublishProcTimeout_ms: ", false);
    GetParamRawPtr(this, "caminfoPublishProcTimeout_ms", caminfoPublishProcTimeout_ms, caminfoPublishProcTimeout_ms, "caminfoPublishProcTimeout_ms: ", false);

    // Add procedure monitors
    this->addProcedureMonitor("_pubImg", std::chrono::nanoseconds((int64_t)(imagePublishProcTimeout_ms * 1e6)));
    this->addProcedureMonitor("_camInfoPubTmCb", std::chrono::nanoseconds((int64_t)(caminfoPublishProcTimeout_ms * 1e6)));

    // Initialize camera
    mCap_.open(mCameraCapID_);
    if (!mCap_.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "[CameraComponent] Unable to open camera.");
        throw std::runtime_error("Unable to open camera");
    }

    if (!cameraAutoSetting)
    {
        mCap_.set(cv::CAP_PROP_FRAME_WIDTH, mCameraWidth_);
        mCap_.set(cv::CAP_PROP_FRAME_HEIGHT, mCameraHeight_);
        mCap_.set(cv::CAP_PROP_FPS, mCameraFPS_);
    }
    else
    {
        mCameraWidth_ = mCap_.get(cv::CAP_PROP_FRAME_WIDTH);
        mCameraHeight_ = mCap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        mCameraFPS_ = mCap_.get(cv::CAP_PROP_FPS);
    }
    RCLCPP_INFO(this->get_logger(), "[CameraComponent] Camera opened successfully.");

    // Start grabbing image
    mGrabImgTh_ = rv2_interfaces::make_unique_thread(&CameraComponent::_grabImg, this);

    // Create publishers
    if (mUseCompression_)
    {
        mEncParam_.push_back(cv::IMWRITE_JPEG_QUALITY);
        mEncParam_.push_back(mCompressionQuality_);
        mCompImgPub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(mTopicName_ + "/image", 10);
    }
    else
    {
        mImgPub_ = this->create_publisher<sensor_msgs::msg::Image>(mTopicName_ + "/image", 10);
    }

    mCamInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(mTopicName_ + "/camera_info", 10);
    mCamInfoPubTm_ = std::make_unique<rv2_interfaces::LiteTimer>(1000, std::bind(&CameraComponent::_camInfoPubTmCb, this));
    mCamInfoPubTm_->start();

    RCLCPP_INFO(this->get_logger(), "[CameraComponent] Constructed.");
}



CameraComponent::~CameraComponent()
{
    mExitF_.store(true);
    mCap_.release();
    RCLCPP_INFO(this->get_logger(), "[CameraComponent] Destructed.");
}



/**
 * ================================================================
 * CameraComponent Private Methods
 * ================================================================
 */



void CameraComponent::_camInfoPubTmCb()
{
    if (mExitF_.load())
    {
        RCLCPP_ERROR(this->get_logger(), "[CameraComponent::_camInfoPubTmCb] Exit flag is set. Exit publishing camera info.");
        return;
    }

    auto msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = mTopicName_ + "_link";
    msg->width = mCameraWidth_;
    msg->height = mCameraHeight_;
    msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    mCamInfoPub_->publish(std::move(msg));
    this->updateProcedureMonitor("_camInfoPubTmCb", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
}



void CameraComponent::_grabImg()
{
    if (mExitF_.load())
    {
        RCLCPP_ERROR(this->get_logger(), "[CameraComponent::_grabImg] Exit flag is set. Exit grabbing image.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[CameraComponent::_grabImg] Start grabbing image.");

    cv::Size pubImgSize(mPublishWidth_, mPublishHeight_);
    cv::Mat frame;

    if (!mCap_.read(frame) || frame.rows <= 0 || frame.cols <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "[CameraComponent::_grabImg] Unable to retrieve image.");
        mExitF_.store(true);
        return;
    }
    if (frame.size() != pubImgSize)
        RCLCPP_WARN(this->get_logger(), "[CameraComponent::_grabImg] The image will be resized into %dx%d. Current image size: %dx%d", 
                    pubImgSize.width, pubImgSize.height, frame.cols, frame.rows);

    while (!mExitF_.load())
    {
        if (!mCap_.read(frame) || frame.rows <= 0 || frame.cols <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "[CameraComponent::_grabImg] Unable to retrieve image.");
            mExitF_.store(true);
            return;
        }
        if (!mCameraUseColor_)
            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        if (frame.size() != pubImgSize)
            cv::resize(frame, frame, pubImgSize);
        this->_pubImg(frame);
        cv::waitKey(1);
    }
}



void CameraComponent::_pubImg(const cv::Mat& img)
{
    if (mExitF_.load())
    {
        RCLCPP_ERROR(this->get_logger(), "[CameraComponent::_pubImg] Exit flag is set. Exit publishing image.");
        return;
    }

    static u_int64_t frame_id = 0;

    if (mUseCompression_)
    {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = mTopicName_ + "_link";

        std::vector<unsigned char> imgVec;
        cv::imencode(".jpg", img, imgVec, mEncParam_);

        msg->format = "jpeg";
        msg->data = imgVec;

        std::lock_guard<std::mutex> imgPubLock(mImgPubMtx_);
        mCompImgPub_->publish(std::move(msg));
    }
    else
    {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = mTopicName_ + "_link";

        msg->height = img.rows;
        msg->width = img.cols;
        if (mCameraUseColor_)
            msg->encoding = sensor_msgs::image_encodings::BGR8;
        else
            msg->encoding = sensor_msgs::image_encodings::MONO8;
        msg->is_bigendian = 0;
        msg->step = img.cols * img.elemSize();
        msg->data = std::vector<uint8_t>(img.data, img.data + img.total() * img.elemSize());

        std::lock_guard<std::mutex> imgPubLock(mImgPubMtx_);
        mImgPub_->publish(std::move(msg));
    }
    this->updateProcedureMonitor("_pubImg", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
}



/**
 * ================================================================
 * CameraComponent Public Methods
 * ================================================================
 */



bool CameraComponent::isExit() const
{
    return mExitF_.load();
}

}// namespace rv2_sensors



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rv2_sensors::CameraComponent)
