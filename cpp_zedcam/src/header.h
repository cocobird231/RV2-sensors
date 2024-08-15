#pragma once
#include <vector>
#include <string>
#include <thread>

#include <chrono>

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

// TF2
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// PC
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Img
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

// CameraInfo
#include <sensor_msgs/distortion_models.hpp>
#include "sensor_msgs/msg/camera_info.hpp"

// IMU
#include "sensor_msgs/msg/imu.hpp"

// Environment
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/vehicle_interfaces.h"
#include "vehicle_interfaces/utils.h"

cv::Mat slMat2cvMat(sl::Mat& input);
sl::Resolution operator*(const sl::Resolution& res, double factor);
void FillCameraInfo(sl::Camera& zed, 
                    std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg, 
                    std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg, 
                    std::atomic<double>& baseline, 
                    std::string leftFrameId, 
                    std::string rightFrameId, 
                    double downsampleFactor, 
                    bool rawParam);

class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string frame_id = "zed";
    double downsampleFactor = 1.0;
    int confidenceThreshold = 30;
    int rgbQuality = 70;

    std::vector<double> camera_topic_ids = { 0 };
    std::vector<double> camera_caps = { 0 };
    double camera_fps = 30.0;
    std::string camera_res = "720p";// Input resolution, "720p" or "1080p"
    std::string camera_svo_path = "";// Path to SVO file.
    bool camera_use_svo = false;// Use SVO file instead of live stream.
    int camera_sensing_mode = 0;// 0: standard mode, 1: fill mode
    int camera_depth_quality = 1;// 0: none, 1: performance, 2: quality, 3: ultra, 4: neural
    int camera_depth_unit = 2;// 0: millimeter, 1: centimeter, 2: meter, 3: inch, 4: foot
    int camera_pc_range = 1;// Point cloud range, 0: short, 1: medium, 2: long, 3: auto
    int camera_pc_res = 0;// Point cloud resolution, 0: high, 1: medium, 2: low
    bool camera_use_color = true;
    bool camera_use_depth = true;
    bool camera_use_pc = true;
    bool camera_use_sens = true;
    double zedThProcTimeout_ms = 200.0;
    double pubRgbProcTimeout_ms = 200.0;
    double pubDepthProcTimeout_ms = 200.0;
    double pubPCProcTimeout_ms = 200.0;
    double pubSensProcTimeout_ms = 200.0;

private:
    void _getParams()
    {
        this->get_parameter("frame_id", this->frame_id);
        this->get_parameter("downsampleFactor", this->downsampleFactor);
        this->get_parameter("confidenceThreshold", this->confidenceThreshold);
        this->get_parameter("rgbQuality", this->rgbQuality);

        this->get_parameter("camera_topic_ids", this->camera_topic_ids);
        this->get_parameter("camera_caps", this->camera_caps);
        this->get_parameter("camera_fps", this->camera_fps);
        this->get_parameter("camera_res", this->camera_res);
        this->get_parameter("camera_svo_path", this->camera_svo_path);
        this->get_parameter("camera_use_svo", this->camera_use_svo);
        this->get_parameter("camera_sensing_mode", this->camera_sensing_mode);
        this->get_parameter("camera_depth_quality", this->camera_depth_quality);
        this->get_parameter("camera_depth_unit", this->camera_depth_unit);
        this->get_parameter("camera_pc_range", this->camera_pc_range);
        this->get_parameter("camera_pc_res", this->camera_pc_res);
        this->get_parameter("camera_use_color", this->camera_use_color);
        this->get_parameter("camera_use_depth", this->camera_use_depth);
        this->get_parameter("camera_use_pc", this->camera_use_pc);
        this->get_parameter("camera_use_sens", this->camera_use_sens);

        this->get_parameter("zedThProcTimeout_ms", this->zedThProcTimeout_ms);
        this->get_parameter("pubRgbProcTimeout_ms", this->pubRgbProcTimeout_ms);
        this->get_parameter("pubDepthProcTimeout_ms", this->pubDepthProcTimeout_ms);
        this->get_parameter("pubPCProcTimeout_ms", this->pubPCProcTimeout_ms);
        this->get_parameter("pubSensProcTimeout_ms", this->pubSensProcTimeout_ms);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("frame_id", this->frame_id);
        this->declare_parameter<double>("downsampleFactor", this->downsampleFactor);
        this->declare_parameter<int>("confidenceThreshold", this->confidenceThreshold);
        this->declare_parameter<int>("rgbQuality", this->rgbQuality);

        this->declare_parameter<std::vector<double> >("camera_topic_ids", this->camera_topic_ids);
        this->declare_parameter<std::vector<double> >("camera_caps", this->camera_caps);
        this->declare_parameter<double>("camera_fps", this->camera_fps);
        this->declare_parameter<std::string>("camera_res", this->camera_res);
        this->declare_parameter<std::string>("camera_svo_path", this->camera_svo_path);
        this->declare_parameter<bool>("camera_use_svo", this->camera_use_svo);
        this->declare_parameter<int>("camera_sensing_mode", this->camera_sensing_mode);
        this->declare_parameter<int>("camera_depth_quality", this->camera_depth_quality);
        this->declare_parameter<int>("camera_depth_unit", this->camera_depth_unit);
        this->declare_parameter<int>("camera_pc_range", this->camera_pc_range);
        this->declare_parameter<int>("camera_pc_res", this->camera_pc_res);
        this->declare_parameter<bool>("camera_use_color", this->camera_use_color);
        this->declare_parameter<bool>("camera_use_depth", this->camera_use_depth);
        this->declare_parameter<bool>("camera_use_pc", this->camera_use_pc);
        this->declare_parameter<bool>("camera_use_sens", this->camera_use_sens);

        this->declare_parameter<double>("zedThProcTimeout_ms", this->zedThProcTimeout_ms);
        this->declare_parameter<double>("pubRgbProcTimeout_ms", this->pubRgbProcTimeout_ms);
        this->declare_parameter<double>("pubDepthProcTimeout_ms", this->pubDepthProcTimeout_ms);
        this->declare_parameter<double>("pubPCProcTimeout_ms", this->pubPCProcTimeout_ms);
        this->declare_parameter<double>("pubSensProcTimeout_ms", this->pubSensProcTimeout_ms);

        this->_getParams();
    }

    void copyParams(const std::shared_ptr<const Params> src)
    {
        this->frame_id = src->frame_id;
        this->downsampleFactor = src->downsampleFactor;
        this->confidenceThreshold = src->confidenceThreshold;
        this->rgbQuality = src->rgbQuality;

        this->camera_topic_ids = src->camera_topic_ids;
        this->camera_caps = src->camera_caps;
        this->camera_fps = src->camera_fps;
        this->camera_res = src->camera_res;
        this->camera_svo_path = src->camera_svo_path;
        this->camera_use_svo = src->camera_use_svo;
        this->camera_sensing_mode = src->camera_sensing_mode;
        this->camera_depth_quality = src->camera_depth_quality;
        this->camera_depth_unit = src->camera_depth_unit;
        this->camera_pc_range = src->camera_pc_range;
        this->camera_pc_res = src->camera_pc_res;
        this->camera_use_color = src->camera_use_color;
        this->camera_use_depth = src->camera_use_depth;
        this->camera_use_pc = src->camera_use_pc;
        this->camera_use_sens = src->camera_use_sens;

        this->zedThProcTimeout_ms = src->zedThProcTimeout_ms;
        this->pubRgbProcTimeout_ms = src->pubRgbProcTimeout_ms;
        this->pubDepthProcTimeout_ms = src->pubDepthProcTimeout_ms;
        this->pubPCProcTimeout_ms = src->pubPCProcTimeout_ms;
        this->pubSensProcTimeout_ms = src->pubSensProcTimeout_ms;
    }
};

class ZEDPublisher : public vehicle_interfaces::PseudoTimeSyncNode
{
private:
    const std::shared_ptr<const Params> params_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage> > leftRgbPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage> > rightRgbPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image> > depthPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2> > pcPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu> > imuPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Temperature> > tempPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::FluidPressure> > pressPub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

    // JPEG compression parameters
    std::vector<int> encodeParam_;

    const std::string parentFrameId_;

    // CameraInfo messages
    std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg_;
    std::atomic<double> baseline_;
    std::mutex camInfoMutex_;
    // CameraInfo publisher
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo> > leftCamInfoPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo> > rightCamInfoPub_;

    // Topic name
    std::string lCamInfoTopicName_;
    std::string lRgbTopicName_;
    std::string rCamInfoTopicName_;
    std::string rRgbTopicName_;
    std::string depthTopicName_;
    std::string pcTopicName_;
    std::string imuTopicName_;
    std::string tempTopicName_;
    std::string pressTopicName_;


public:
    ZEDPublisher(const std::shared_ptr<const Params> params, std::string parent, sl::Camera& zed, const geometry_msgs::msg::Transform& tf = geometry_msgs::msg::Transform()) : 
        PseudoTimeSyncNode(params->nodeName), 
        rclcpp::Node(params->nodeName), 
        params_(params), 
        parentFrameId_(parent)
    {
        this->encodeParam_.push_back(cv::IMWRITE_JPEG_QUALITY);
        this->encodeParam_.push_back(this->params_->rgbQuality);

        this->lCamInfoTopicName_ = this->params_->frame_id + "/left/camera_info";
        this->lRgbTopicName_ = this->params_->frame_id + "/left/rgb";
        this->rCamInfoTopicName_ = this->params_->frame_id + "/right/camera_info";
        this->rRgbTopicName_ = this->params_->frame_id + "/right/rgb";
        this->depthTopicName_ = this->params_->frame_id + "/depth";
        this->pcTopicName_ = this->params_->frame_id + "/point_cloud";
        this->imuTopicName_ = this->params_->frame_id + "/imu";
        this->tempTopicName_ = this->params_->frame_id + "/temperature";
        this->pressTopicName_ = this->params_->frame_id + "/pressure";

        this->leftCamInfoMsg_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
        this->rightCamInfoMsg_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
        FillCameraInfo(zed, this->leftCamInfoMsg_, this->rightCamInfoMsg_, this->baseline_, this->lRgbTopicName_, this->rRgbTopicName_, this->params_->downsampleFactor, false);
        this->leftCamInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(this->lCamInfoTopicName_, 10);
        this->rightCamInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(this->rCamInfoTopicName_, 10);

        this->leftRgbPub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(this->lRgbTopicName_, 10);
        this->rightRgbPub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(this->rRgbTopicName_, 10);
        this->depthPub_ = this->create_publisher<sensor_msgs::msg::Image>(this->depthTopicName_, 10);
        this->pcPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->pcTopicName_, 10);
        this->imuPub_ = this->create_publisher<sensor_msgs::msg::Imu>(this->imuTopicName_, 10);
        this->tempPub_ = this->create_publisher<sensor_msgs::msg::Temperature>(this->tempTopicName_, 10);
        this->pressPub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(this->pressTopicName_, 10);

        this->tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        RCLCPP_INFO(this->get_logger(), "[ZEDPublisher] Constructed");
    }

    void pubLeftRgb(sl::Mat& img, sl::Timestamp ts)
    {
        if (this->leftRgbPub_->get_subscription_count() == 0)
            return;

        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.frame_id = this->lRgbTopicName_;
        if (ts.getNanoseconds() == 0)
            msg->header.stamp = this->getTimestamp();
        else
            msg->header.stamp = rclcpp::Time(ts.getNanoseconds());

        msg->format = "jpeg";

        std::vector<unsigned char> pubRGBImgVec;
        cv::imencode(".jpg", slMat2cvMat(img), pubRGBImgVec, this->encodeParam_);
        msg->data = pubRGBImgVec;

        this->leftRgbPub_->publish(std::move(msg));
    }

    void pubRightRgb(sl::Mat& img, sl::Timestamp ts)
    {
        if (this->rightRgbPub_->get_subscription_count() == 0)
            return;

        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.frame_id = this->rRgbTopicName_;
        if (ts.getNanoseconds() == 0)
            msg->header.stamp = this->getTimestamp();
        else
            msg->header.stamp = rclcpp::Time(ts.getNanoseconds());

        msg->format = "jpeg";

        std::vector<unsigned char> pubRGBImgVec;
        cv::imencode(".jpg", slMat2cvMat(img), pubRGBImgVec, this->encodeParam_);
        msg->data = pubRGBImgVec;

        this->rightRgbPub_->publish(std::move(msg));
    }

    void pubDepth(sl::Mat& img, sl::Timestamp ts)
    {
        if (this->depthPub_->get_subscription_count() == 0)
            return;

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.frame_id = this->depthTopicName_;
        if (ts.getNanoseconds() == 0)
            msg->header.stamp = this->getTimestamp();
        else
            msg->header.stamp = rclcpp::Time(ts.getNanoseconds());

        msg->height = img.getHeight();
        msg->width = img.getWidth();
        msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

        uint32_t step = img.getStepBytes();
        size_t size = step * img.getHeight();
        uint8_t *data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float1>());

        msg->step = step;
        msg->data = std::vector<uint8_t>(data_ptr, data_ptr + size);

        this->depthPub_->publish(std::move(msg));
    }

    void pubPC(sl::Mat& pc, sl::Pose& pose, sl::Timestamp ts)
    {
        if (this->pcPub_->get_subscription_count() == 0)
            return;

        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        msg->header.frame_id = this->pcTopicName_;
        if (ts.getNanoseconds() == 0)
            msg->header.stamp = this->getTimestamp();
        else
            msg->header.stamp = rclcpp::Time(ts.getNanoseconds());

        msg->height = pc.getHeight();
        msg->width = pc.getWidth();

        sensor_msgs::PointCloud2Modifier modifier(*(msg.get()));
        modifier.setPointCloud2Fields(4, 
            "x", 1, sensor_msgs::msg::PointField::FLOAT32, 
            "y", 1, sensor_msgs::msg::PointField::FLOAT32, 
            "z", 1, sensor_msgs::msg::PointField::FLOAT32, 
            "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);

        sl::Vector4<float> *cpu_cloud = pc.getPtr<sl::float4>();
        float *ptCloudPtr = reinterpret_cast<float *>(&msg->data[0]);
        memcpy(ptCloudPtr, reinterpret_cast<float *>(cpu_cloud), msg->height * msg->width * 4 * sizeof(float));

        msg->is_bigendian = false;
        msg->is_dense = false;

        this->pcPub_->publish(std::move(msg));

        // ----> Publish TF
        auto orient = pose.getOrientation();
        auto trans = pose.getTranslation();
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = this->getTimestamp();

        transformStamped.header.frame_id = this->parentFrameId_;
        transformStamped.child_frame_id = this->pcTopicName_;

        transformStamped.transform.rotation.x = orient.ox;
        transformStamped.transform.rotation.y = orient.oy;
        transformStamped.transform.rotation.z = orient.oz;
        transformStamped.transform.rotation.w = orient.ow;

        transformStamped.transform.translation.x = trans.x;
        transformStamped.transform.translation.y = trans.y;
        transformStamped.transform.translation.z = trans.z;

        this->tfBroadcaster_->sendTransform(transformStamped);
        // <---- Publish TF
    }

    void pubImu(sl::SensorsData& imuData)
    {
        if (this->imuPub_->get_subscription_count() == 0)
            return;

        auto msg = std::make_unique<sensor_msgs::msg::Imu>();
        msg->header.stamp = this->getTimestamp();
        msg->header.frame_id = this->imuTopicName_;

        msg->angular_velocity.x = imuData.imu.angular_velocity[0];
        msg->angular_velocity.y = imuData.imu.angular_velocity[1];
        msg->angular_velocity.z = imuData.imu.angular_velocity[2];
        // msg->angular_velocity_covariance = imuData.imu.angular_velocity_covariance;
        msg->linear_acceleration.x = imuData.imu.linear_acceleration[0];
        msg->linear_acceleration.y = imuData.imu.linear_acceleration[1];
        msg->linear_acceleration.z = imuData.imu.linear_acceleration[2];
        // msg->linear_acceleration_covariance = imuData.imu.linear_acceleration_covariance;

        auto imuPose = imuData.imu.pose.getOrientation();
        msg->orientation.x = imuPose.ox;
        msg->orientation.y = imuPose.oy;
        msg->orientation.z = imuPose.oz;
        msg->orientation.w = imuPose.ow;
        // msg->orientation_covariance = imuData.imu.pose.getOrientationCovariance();

        this->imuPub_->publish(std::move(msg));
    }

    void pubEnv(sl::SensorsData& envData)
    {
        float temp, press;
        envData.temperature.get(sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT, temp);
        press = envData.barometer.pressure;
        auto tempMsg = std::make_unique<sensor_msgs::msg::Temperature>();
        tempMsg->header.stamp = this->getTimestamp();
        tempMsg->header.frame_id = this->tempTopicName_;
        tempMsg->temperature = (double)temp;
        tempMsg->variance = 0.0;

        auto pressMsg = std::make_unique<sensor_msgs::msg::FluidPressure>();
        pressMsg->header.stamp = this->getTimestamp();
        pressMsg->header.frame_id = this->pressTopicName_;
        pressMsg->fluid_pressure = (double)press;
        pressMsg->variance = 0.0;

        this->tempPub_->publish(std::move(tempMsg));
        this->pressPub_->publish(std::move(pressMsg));
    }

    void pubCamInfo()
    {
        std::lock_guard<std::mutex> lock(this->camInfoMutex_);
        this->leftCamInfoMsg_->header.stamp = this->getTimestamp();
        this->rightCamInfoMsg_->header.stamp = this->getTimestamp();
        this->leftCamInfoPub_->publish(*(this->leftCamInfoMsg_));
        this->rightCamInfoPub_->publish(*(this->rightCamInfoMsg_));
    }

    sensor_msgs::msg::CameraInfo getLeftCamInfo()
    {
        std::lock_guard<std::mutex> lock(this->camInfoMutex_);
        return *(this->leftCamInfoMsg_);
    }

    sensor_msgs::msg::CameraInfo getRightCamInfo()
    {
        std::lock_guard<std::mutex> lock(this->camInfoMutex_);
        return *(this->rightCamInfoMsg_);
    }

    double getBaseline() const
    {
        return this->baseline_;
    }

    std::shared_ptr<const Params> getParams() const
    {
        return this->params_;
    }
};


enum ZEDNodeErrCode
{
    Success, 
    ZEDPublisherAlreadyExists
};


class ZEDNode : public vehicle_interfaces::VehicleServiceNode
{
private:
    const std::shared_ptr<const Params> params_;
    std::map<int, std::shared_ptr<ZEDPublisher> > zedPubs_;
    std::map<int, rclcpp::executors::SingleThreadedExecutor::SharedPtr> executors_;
    std::map<int, vehicle_interfaces::unique_thread> threads_;

public:
    ZEDNode(const std::shared_ptr<const Params> params) : 
        vehicle_interfaces::VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        params_(params)
    {
        RCLCPP_INFO(this->get_logger(), "[ZEDNode] Constructed");
    }

    ~ZEDNode()
    {
        this->removeAllZEDPublishers();
        RCLCPP_INFO(this->get_logger(), "[ZEDNode] Destructed");
    }

    ZEDNodeErrCode addZEDPublisher(int id, sl::Camera& zed)
    {
        if (this->zedPubs_.find(id) != this->zedPubs_.end())
            return ZEDNodeErrCode::ZEDPublisherAlreadyExists;
        std::string suffix = "_" + std::to_string(id);
        auto tmpParams = std::make_shared<Params>(this->params_->nodeName + "_params" + suffix);
        tmpParams->copyParams(this->params_);
        tmpParams->nodeName = this->params_->nodeName + suffix;
        tmpParams->frame_id = this->params_->frame_id + suffix;

        this->zedPubs_[id] = std::make_shared<ZEDPublisher>(tmpParams, this->params_->nodeName, zed);
        this->executors_[id] = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        this->executors_[id]->add_node(this->zedPubs_[id]);
        this->threads_[id] = vehicle_interfaces::make_unique_thread(vehicle_interfaces::SpinExecutor, this->executors_[id], tmpParams->nodeName, 1000);

        if (tmpParams->camera_use_color)
            this->addProcedureMonitor("ZED" + suffix + "_pubRgb", std::chrono::nanoseconds((int64_t)(this->params_->pubRgbProcTimeout_ms * 1e6)));
        if (tmpParams->camera_use_depth)
            this->addProcedureMonitor("ZED" + suffix + "_pubDepth", std::chrono::nanoseconds((int64_t)(this->params_->pubDepthProcTimeout_ms * 1e6)));
        if (tmpParams->camera_use_pc)
            this->addProcedureMonitor("ZED" + suffix + "_pubPC", std::chrono::nanoseconds((int64_t)(this->params_->pubPCProcTimeout_ms * 1e6)));
        if (tmpParams->camera_use_sens)
            this->addProcedureMonitor("ZED" + suffix + "_pubSens", std::chrono::nanoseconds((int64_t)(this->params_->pubSensProcTimeout_ms * 1e6)));
        return ZEDNodeErrCode::Success;
    }

    void removeZEDPublisher(int id)
    {
        if (this->zedPubs_.find(id) != this->zedPubs_.end())
        {
            this->executors_[id]->cancel();
            this->threads_.erase(id);
            this->executors_.erase(id);
            this->zedPubs_.erase(id);
            RCLCPP_WARN(this->get_logger(), "Removed ZEDPublisher %d", id);
        }
    }

    void removeAllZEDPublishers()
    {
        for (auto it = this->zedPubs_.begin(); it != this->zedPubs_.end(); it++)
        {
            this->executors_[it->first]->cancel();
            this->threads_.erase(it->first);
            this->executors_.erase(it->first);
            RCLCPP_WARN(this->get_logger(), "Removed ZEDPublisher %d", it->first);
        }
        this->zedPubs_.clear();
    }

    void pubLeftRgb(int id, sl::Mat& img, const sl::Timestamp& ts = sl::Timestamp(0))
    {
        if (this->zedPubs_.find(id) != this->zedPubs_.end())
        {
            this->zedPubs_[id]->pubLeftRgb(img, ts);
            if (this->params_->camera_use_color)
                this->updateProcedureMonitor("ZED_" + std::to_string(id) + "_pubRgb", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
        }
    }

    void pubRightRgb(int id, sl::Mat& img, const sl::Timestamp& ts = sl::Timestamp(0))
    {
        if (this->zedPubs_.find(id) != this->zedPubs_.end())
        {
            this->zedPubs_[id]->pubRightRgb(img, ts);
            if (this->params_->camera_use_color)
                this->updateProcedureMonitor("ZED_" + std::to_string(id) + "_pubRgb", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
        }
    }

    void pubDepth(int id, sl::Mat& img, const sl::Timestamp& ts = sl::Timestamp(0))
    {
        if (this->zedPubs_.find(id) != this->zedPubs_.end())
        {
            this->zedPubs_[id]->pubDepth(img, ts);
            if (this->params_->camera_use_depth)
                this->updateProcedureMonitor("ZED_" + std::to_string(id) + "_pubDepth", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
        }
    }

    void pubPC(int id, sl::Mat& pc, sl::Pose& pose, const sl::Timestamp& ts = sl::Timestamp(0))
    {
        if (this->zedPubs_.find(id) != this->zedPubs_.end())
        {
            this->zedPubs_[id]->pubPC(pc, pose, ts);
            if (this->params_->camera_use_pc)
                this->updateProcedureMonitor("ZED_" + std::to_string(id) + "_pubPC", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
        }
    }

    void pubCamInfo(int id)
    {
        if (this->zedPubs_.find(id) != this->zedPubs_.end())
            this->zedPubs_[id]->pubCamInfo();
    }

    bool getCamInfo(int id, sensor_msgs::msg::CameraInfo& lCamInfo, sensor_msgs::msg::CameraInfo& rCamInfo, double& baseline)
    {
        if (this->zedPubs_.find(id) != this->zedPubs_.end())
        {
            lCamInfo = this->zedPubs_[id]->getLeftCamInfo();
            rCamInfo = this->zedPubs_[id]->getRightCamInfo();
            baseline = this->zedPubs_[id]->getBaseline();
            return true;
        }
        return false;
    }

    bool pubImu(int id, sl::SensorsData& imuData)
    {
        if (this->zedPubs_.find(id) != this->zedPubs_.end())
        {
            this->zedPubs_[id]->pubImu(imuData);
            if (this->params_->camera_use_sens)
                this->updateProcedureMonitor("ZED_" + std::to_string(id) + "_pubSens", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
            return true;
        }
        return false;
    }

    bool pubEnv(int id, sl::SensorsData& envData)
    {
        if (this->zedPubs_.find(id) != this->zedPubs_.end())
        {
            this->zedPubs_[id]->pubEnv(envData);
            if (this->params_->camera_use_sens)
                this->updateProcedureMonitor("ZED_" + std::to_string(id) + "_pubSens", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
            return true;
        }
        return false;
    }

    bool getParams(int id, std::shared_ptr<const Params>& params)
    {
        if (this->zedPubs_.find(id) != this->zedPubs_.end())
        {
            params = this->zedPubs_[id]->getParams();
            return true;
        }
        return false;
    }
};



cv::Mat slMat2cvMat(sl::Mat& input)
{
	// Mapping between MAT_TYPE and CV_TYPE
	int cv_type = -1;
	switch (input.getDataType())
    {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
	}

	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

sl::Resolution operator*(const sl::Resolution& res, double factor)
{
    return sl::Resolution((int)(res.width * factor), (int)(res.height * factor));
}

void FillCameraInfo(sl::Camera& zed, 
                    std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg, 
                    std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg, 
                    std::atomic<double>& baseline, 
                    std::string leftFrameId, 
                    std::string rightFrameId, 
                    double downsampleFactor, 
                    bool rawParam)
{
    sl::CalibrationParameters zedParam;
    sl::Resolution mMatResol = zed.getCameraInformation().camera_configuration.resolution * downsampleFactor;
    sl::MODEL model = zed.getCameraInformation().camera_model;
    if (rawParam)
    {
        zedParam = zed.getCameraInformation(mMatResol).camera_configuration.calibration_parameters_raw;
    }
    else
    {
        zedParam = zed.getCameraInformation(mMatResol).camera_configuration.calibration_parameters;
    }

    baseline = zedParam.getCameraBaseline();
    printf("[FillCameraInfo] Baseline: %f\n", baseline.load());

    // ----> Distortion models
    // ZED SDK params order: [ k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]
    // Radial (k1, k2, k3, k4, k5, k6), Tangential (p1,p2) and Prism (s1, s2, s3, s4) distortion.
    // Prism not currently used.

    // ROS2 order (OpenCV) -> k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
    switch (model)
    {
        case sl::MODEL::ZED: // PLUMB_BOB
            leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            leftCamInfoMsg->d.resize(5);
            rightCamInfoMsg->d.resize(5);
            leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
            leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
            leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
            leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
            leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
            rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
            rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
            rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
            rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
            rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
            break;

        case sl::MODEL::ZED2:  // RATIONAL_POLYNOMIAL
        case sl::MODEL::ZED2i:  // RATIONAL_POLYNOMIAL
        case sl::MODEL::ZED_X:  // RATIONAL_POLYNOMIAL
        case sl::MODEL::ZED_XM:  // RATIONAL_POLYNOMIAL
            leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
            rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
            leftCamInfoMsg->d.resize(8);
            rightCamInfoMsg->d.resize(8);
            leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
            leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
            leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
            leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
            leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
            leftCamInfoMsg->d[5] = zedParam.left_cam.disto[5];    // k4
            leftCamInfoMsg->d[6] = zedParam.left_cam.disto[6];    // k5
            leftCamInfoMsg->d[7] = zedParam.left_cam.disto[7];    // k6
            rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
            rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
            rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
            rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
            rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
            rightCamInfoMsg->d[5] = zedParam.right_cam.disto[5];  // k4
            rightCamInfoMsg->d[6] = zedParam.right_cam.disto[6];  // k5
            rightCamInfoMsg->d[7] = zedParam.right_cam.disto[7];  // k6
            break;

        case sl::MODEL::ZED_M:
            if (zedParam.left_cam.disto[5] != 0 && // k4!=0
                zedParam.right_cam.disto[2] == 0 && // p1==0
                zedParam.right_cam.disto[3] == 0) // p2==0
            {
                leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
                rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;

                leftCamInfoMsg->d.resize(4);
                rightCamInfoMsg->d.resize(4);
                leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
                leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
                leftCamInfoMsg->d[2] = zedParam.left_cam.disto[4];    // k3
                leftCamInfoMsg->d[3] = zedParam.left_cam.disto[5];    // k4
                rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
                rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
                rightCamInfoMsg->d[2] = zedParam.right_cam.disto[4];  // k3
                rightCamInfoMsg->d[3] = zedParam.right_cam.disto[5];  // k4
            }
            else
            {
                leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
                rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
                leftCamInfoMsg->d.resize(5);
                rightCamInfoMsg->d.resize(5);
                leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];  // k1
                leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];  // k2
                leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];  // p1
                leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];  // p2
                leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];  // k3
                rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0]; // k1
                rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1]; // k2
                rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2]; // p1
                rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3]; // p2
                rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4]; // k3
            }
    }
    printf("[FillCameraInfo] Distortion model: %s\n", leftCamInfoMsg->distortion_model.c_str());

    leftCamInfoMsg->k.fill(0.0);
    rightCamInfoMsg->k.fill(0.0);
    leftCamInfoMsg->k[0] = static_cast<double>(zedParam.left_cam.fx);
    leftCamInfoMsg->k[2] = static_cast<double>(zedParam.left_cam.cx);
    leftCamInfoMsg->k[4] = static_cast<double>(zedParam.left_cam.fy);
    leftCamInfoMsg->k[5] = static_cast<double>(zedParam.left_cam.cy);
    leftCamInfoMsg->k[8] = 1.0;
    rightCamInfoMsg->k[0] = static_cast<double>(zedParam.right_cam.fx);
    rightCamInfoMsg->k[2] = static_cast<double>(zedParam.right_cam.cx);
    rightCamInfoMsg->k[4] = static_cast<double>(zedParam.right_cam.fy);
    rightCamInfoMsg->k[5] = static_cast<double>(zedParam.right_cam.cy);
    rightCamInfoMsg->k[8] = 1.0;
    leftCamInfoMsg->r.fill(0.0);
    rightCamInfoMsg->r.fill(0.0);

    for (size_t i = 0; i < 3; i++)
    {
        // identity
        rightCamInfoMsg->r[i + i * 3] = 1;
        leftCamInfoMsg->r[i + i * 3] = 1;
    }

    if (rawParam)
    {
        // ROS frame (X forward, Z up, Y left)
        for (int i = 0; i < 9; i++)
        {
            rightCamInfoMsg->r[i] = zedParam.stereo_transform.getRotationMatrix().r[i];
        }
    }

    leftCamInfoMsg->p.fill(0.0);
    rightCamInfoMsg->p.fill(0.0);
    leftCamInfoMsg->p[0] = static_cast<double>(zedParam.left_cam.fx);
    leftCamInfoMsg->p[2] = static_cast<double>(zedParam.left_cam.cx);
    leftCamInfoMsg->p[5] = static_cast<double>(zedParam.left_cam.fy);
    leftCamInfoMsg->p[6] = static_cast<double>(zedParam.left_cam.cy);
    leftCamInfoMsg->p[10] = 1.0;
    // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
    rightCamInfoMsg->p[3] = static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
    rightCamInfoMsg->p[0] = static_cast<double>(zedParam.right_cam.fx);
    rightCamInfoMsg->p[2] = static_cast<double>(zedParam.right_cam.cx);
    rightCamInfoMsg->p[5] = static_cast<double>(zedParam.right_cam.fy);
    rightCamInfoMsg->p[6] = static_cast<double>(zedParam.right_cam.cy);
    rightCamInfoMsg->p[10] = 1.0;
    leftCamInfoMsg->width = rightCamInfoMsg->width = static_cast<uint32_t>(mMatResol.width);
    leftCamInfoMsg->height = rightCamInfoMsg->height = static_cast<uint32_t>(mMatResol.height);
    leftCamInfoMsg->header.frame_id = leftFrameId;
    rightCamInfoMsg->header.frame_id = rightFrameId;
}
