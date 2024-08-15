#pragma once
#include <vector>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/timer.h"
#include "vehicle_interfaces/vehicle_interfaces.h"
// Img
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
// CameraInfo
#include <sensor_msgs/distortion_models.hpp>
#include "sensor_msgs/msg/camera_info.hpp"

#include <opencv2/opencv.hpp>

class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string topic_Webcam_topicName = "webcam_0";
    float topic_Webcam_pubInterval_s = 0.03;
    int topic_Webcam_width = 1920;
    int topic_Webcam_height = 1080;
    bool topic_use_compression = true;
    int topic_compression_quality = 80;
    int camera_cap_id = 0;
    float camera_fps = 30.0;
    int camera_width = 1920;
    int camera_height = 1080;
    bool camera_use_color = true;
    float cameraProcTimeout_ms = 66.0;
    float pubImageProcTimeout_ms = 200.0;
    float pubCamInfoProcTimeout_ms = 2000.0;

private:
    void _getParams()
    {
        this->get_parameter("topic_Webcam_topicName", this->topic_Webcam_topicName);
        this->get_parameter("topic_Webcam_pubInterval_s", this->topic_Webcam_pubInterval_s);
        this->get_parameter("topic_Webcam_width", this->topic_Webcam_width);
        this->get_parameter("topic_Webcam_height", this->topic_Webcam_height);
        this->get_parameter("topic_use_compression", this->topic_use_compression);
        this->get_parameter("topic_compression_quality", this->topic_compression_quality);
        this->get_parameter("camera_cap_id", this->camera_cap_id);
        this->get_parameter("camera_fps", this->camera_fps);
        this->get_parameter("camera_width", this->camera_width);
        this->get_parameter("camera_height", this->camera_height);
        this->get_parameter("camera_use_color", this->camera_use_color);
        this->get_parameter("cameraProcTimeout_ms", this->cameraProcTimeout_ms);
        this->get_parameter("pubImageProcTimeout_ms", this->pubImageProcTimeout_ms);
        this->get_parameter("pubCamInfoProcTimeout_ms", this->pubCamInfoProcTimeout_ms);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("topic_Webcam_topicName", this->topic_Webcam_topicName);
        this->declare_parameter<float>("topic_Webcam_pubInterval_s", this->topic_Webcam_pubInterval_s);
        this->declare_parameter<int>("topic_Webcam_width", this->topic_Webcam_width);
        this->declare_parameter<int>("topic_Webcam_height", this->topic_Webcam_height);
        this->declare_parameter<bool>("topic_use_compression", this->topic_use_compression);
        this->declare_parameter<int>("topic_compression_quality", this->topic_compression_quality);
        this->declare_parameter<int>("camera_cap_id", this->camera_cap_id);
        this->declare_parameter<float>("camera_fps", this->camera_fps);
        this->declare_parameter<int>("camera_width", this->camera_width);
        this->declare_parameter<int>("camera_height", this->camera_height);
        this->declare_parameter<bool>("camera_use_color", this->camera_use_color);
        this->declare_parameter<float>("cameraProcTimeout_ms", this->cameraProcTimeout_ms);
        this->declare_parameter<float>("pubImageProcTimeout_ms", this->pubImageProcTimeout_ms);
        this->declare_parameter<float>("pubCamInfoProcTimeout_ms", this->pubCamInfoProcTimeout_ms);
        this->_getParams();
    }
};


class RGBImagePublisher : public vehicle_interfaces::VehicleServiceNode
{
private:
    const std::shared_ptr<const Params> params_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoPub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr cImgPub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPub_;
    std::mutex imgPubMtx_;

    std::vector<int> encodeParam_;// OpenCV image encoding parameters

    std::unique_ptr<vehicle_interfaces::LiteTimer> camInfoPubTm_;

private:
    void _camInfoPubTmCb()
    {
        auto msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
        msg->header.stamp = this->getTimestamp();
        msg->header.frame_id = this->params_->topic_Webcam_topicName;
        msg->width = this->params_->camera_width;
        msg->height = this->params_->camera_height;
        msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        this->camInfoPub_->publish(std::move(msg));
        this->updateProcedureMonitor("_camInfoPubTmCb", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
    }

public:
    RGBImagePublisher(const std::shared_ptr<const Params>& params) : 
        vehicle_interfaces::VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        params_(params)
    {
        if (params->topic_use_compression)
        {
            this->encodeParam_.push_back(cv::IMWRITE_JPEG_QUALITY);
            this->encodeParam_.push_back(this->params_->topic_compression_quality);
            this->cImgPub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(params->topic_Webcam_topicName + "/rgb", 10);
        }
        else
        {
            this->imgPub_ = this->create_publisher<sensor_msgs::msg::Image>(params->topic_Webcam_topicName + "/rgb", 10);
        }

        this->camInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(params->topic_Webcam_topicName + "/camera_info", 10);
        this->camInfoPubTm_ = std::make_unique<vehicle_interfaces::LiteTimer>(1000, std::bind(&RGBImagePublisher::_camInfoPubTmCb, this));
        this->camInfoPubTm_->start();

        this->addProcedureMonitor("pubImage", std::chrono::nanoseconds((int64_t)(params->pubImageProcTimeout_ms * 1e6)));
        this->addProcedureMonitor("_camInfoPubTmCb", std::chrono::nanoseconds((int64_t)(params->pubCamInfoProcTimeout_ms * 1e6)));
    }

    void pubImage(const cv::Mat& img)
    {
        static u_int64_t frame_id = 0;

        if (this->params_->topic_use_compression)
        {
            auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            msg->header.stamp = this->getTimestamp();
            msg->header.frame_id = this->params_->topic_Webcam_topicName;

            std::vector<unsigned char> imgVec;
            cv::imencode(".jpg", img, imgVec, this->encodeParam_);

            msg->format = "jpeg";
            msg->data = imgVec;

            std::lock_guard<std::mutex> imgPubLock(this->imgPubMtx_);
            this->cImgPub_->publish(std::move(msg));
        }
        else
        {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = this->getTimestamp();
            msg->header.frame_id = this->params_->topic_Webcam_topicName;

            msg->height = img.rows;
            msg->width = img.cols;
            msg->encoding = sensor_msgs::image_encodings::BGR8;
            msg->is_bigendian = 0;
            msg->step = img.cols * img.elemSize();
            msg->data = std::vector<uint8_t>(img.data, img.data + img.total() * img.elemSize());

            std::lock_guard<std::mutex> imgPubLock(this->imgPubMtx_);
            this->imgPub_->publish(std::move(msg));
        }
        this->updateProcedureMonitor("pubImage", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
    }
};
