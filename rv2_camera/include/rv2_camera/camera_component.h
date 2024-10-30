#pragma once
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rv2_interfaces/timer.h"
#include "rv2_interfaces/rv2_interfaces.h"
// Img
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
// CameraInfo
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <opencv2/opencv.hpp>

#define DEFAULT_CAMERA_NODENAME "rv2_camera_default_node"
#define DEFAULT_CAMERA_TOPIC "camera_default"

namespace rv2_sensors
{

class CameraComponent : public rv2_interfaces::VehicleServiceNode
{
private:
    // Publish parameters
    std::string mTopicName_ = DEFAULT_CAMERA_TOPIC;
    int mPublishWidth_ = 1920;
    int mPublishHeight_ = 1080;
    bool mUseCompression_ = false;
    int mCompressionQuality_ = 80;

    // Camera parameters
    int mCameraCapID_ = 0;
    double mCameraFPS_ = 30.0;
    int mCameraWidth_ = 640;
    int mCameraHeight_ = 480;
    bool mCameraUseColor_ = true;

    cv::VideoCapture mCap_;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mCamInfoPub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr mCompImgPub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImgPub_;
    std::mutex mImgPubMtx_;

    std::vector<int> mEncParam_;// OpenCV image encoding parameters

    std::unique_ptr<rv2_interfaces::LiteTimer> mCamInfoPubTm_;
    rv2_interfaces::unique_thread mGrabImgTh_;

    std::atomic<bool> mExitF_;

public:
    CameraComponent(const rclcpp::NodeOptions & options);

    ~CameraComponent();

private:
    void _camInfoPubTmCb();

    void _grabImg();

    void _pubImg(const cv::Mat& img);

public:
    bool isExit() const;
};

}
