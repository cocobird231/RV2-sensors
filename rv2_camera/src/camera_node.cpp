#include <rv2_camera/camera_node.h>

void CameraNode::_camInfoPubTmCb()
{
    if (mExitF_.load())
    {
        RCLCPP_ERROR(this->get_logger(), "[CameraNode::_camInfoPubTmCb] Exit flag is set. Exit publishing camera info.");
        return;
    }

    auto msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    msg->header.stamp = this->getTimestamp();
    msg->header.frame_id = mTopicName_ + "_link";
    msg->width = mCameraWidth_;
    msg->height = mCameraHeight_;
    msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    mCamInfoPub_->publish(std::move(msg));
    this->updateProcedureMonitor("_camInfoPubTmCb", rv2_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
}

void CameraNode::_grabImg()
{
    if (mExitF_.load())
    {
        RCLCPP_ERROR(this->get_logger(), "[CameraNode::_grabImg] Exit flag is set. Exit grabbing image.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[CameraNode::_grabImg] Start grabbing image.");

    cv::Size pubImgSize(mPublishWidth_, mPublishHeight_);
    cv::Mat frame;

    if (!mCap_.read(frame) || frame.rows <= 0 || frame.cols <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "[CameraNode::_grabImg] Unable to retrieve image.");
        mExitF_.store(true);
        return;
    }
    if (frame.size() != pubImgSize)
        RCLCPP_WARN(this->get_logger(), "[CameraNode::_grabImg] The image will be resized into %dx%d. Current image size: %dx%d", 
                    pubImgSize.width, pubImgSize.height, frame.cols, frame.rows);

    while (!mExitF_.load())
    {
        if (!mCap_.read(frame) || frame.rows <= 0 || frame.cols <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "[CameraNode::_grabImg] Unable to retrieve image.");
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

void CameraNode::_pubImg(const cv::Mat& img)
{
    if (mExitF_.load())
    {
        RCLCPP_ERROR(this->get_logger(), "[CameraNode::_pubImg] Exit flag is set. Exit publishing image.");
        return;
    }

    static u_int64_t frame_id = 0;

    if (mUseCompression_)
    {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->getTimestamp();
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
        msg->header.stamp = this->getTimestamp();
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

CameraNode::CameraNode(const std::shared_ptr<const rv2_interfaces::GenericParams>& gParams) : 
    rv2_interfaces::VehicleServiceNode(gParams), 
    rclcpp::Node(gParams->nodeName), 
    mExitF_(false)
{
    // Get parameters
    this->getParam("topicName", mTopicName_, mTopicName_, "topicName: ", false);
    this->getParam("publishWidth", mPublishWidth_, mPublishWidth_, "publishWidth: ", false);
    this->getParam("publishHeight", mPublishHeight_, mPublishHeight_, "publishHeight: ", false);
    this->getParam("useCompression", mUseCompression_, mUseCompression_, "useCompression: ", false);
    this->getParam("compressionQuality", mCompressionQuality_, mCompressionQuality_, "compressionQuality: ", false);

    bool cameraAutoSetting = true;
    this->getParam("cameraCapID", mcameraCapID_, mcameraCapID_, "cameraCapID: ", false);
    this->getParam("cameraAutoSetting", cameraAutoSetting, cameraAutoSetting, "cameraAutoSetting: ", false);
    this->getParam("cameraFPS", mCameraFPS_, mCameraFPS_, "cameraFPS: ", false);
    this->getParam("cameraWidth", mCameraWidth_, mCameraWidth_, "cameraWidth: ", false);
    this->getParam("cameraHeight", mCameraHeight_, mCameraHeight_, "cameraHeight: ", false);
    this->getParam("cameraUseColor", mCameraUseColor_, mCameraUseColor_, "cameraUseColor: ", false);

    double imagePublishProcTimeout_ms = 200.0;
    double caminfoPublishProcTimeout_ms = 2000.0;
    this->getParam("imagePublishProcTimeout_ms", imagePublishProcTimeout_ms, imagePublishProcTimeout_ms, "imagePublishProcTimeout_ms: ", false);
    this->getParam("caminfoPublishProcTimeout_ms", caminfoPublishProcTimeout_ms, caminfoPublishProcTimeout_ms, "caminfoPublishProcTimeout_ms: ", false);

    // Add procedure monitors
    this->addProcedureMonitor("_pubImg", std::chrono::nanoseconds((int64_t)(imagePublishProcTimeout_ms * 1e6)));
    this->addProcedureMonitor("_camInfoPubTmCb", std::chrono::nanoseconds((int64_t)(caminfoPublishProcTimeout_ms * 1e6)));

    // Initialize camera
    mCap_.open(mcameraCapID_);
    if (!mCap_.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "[CameraNode] Unable to open camera.");
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
    RCLCPP_INFO(this->get_logger(), "[CameraNode] Camera opened successfully.");

    // Start grabbing image
    mGrabImgTh_ = rv2_interfaces::make_unique_thread(&CameraNode::_grabImg, this);

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
    mCamInfoPubTm_ = std::make_unique<rv2_interfaces::LiteTimer>(1000, std::bind(&CameraNode::_camInfoPubTmCb, this));
    mCamInfoPubTm_->start();

    RCLCPP_INFO(this->get_logger(), "[CameraNode] Constructed.");
}

CameraNode::~CameraNode()
{
    mExitF_.store(true);
    mCap_.release();
    RCLCPP_INFO(this->get_logger(), "[CameraNode] Destructed.");
}

bool CameraNode::isExit() const
{
    return mExitF_.load();
}
