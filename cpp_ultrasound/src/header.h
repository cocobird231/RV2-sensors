#pragma once
#include <vector>
#include <array>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/timer.h"
#include "vehicle_interfaces/vehicle_interfaces.h"

// TTY
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define ULTRASOUND_MODULE_SIZE 4
#define DEG2RAD(x) ((x) * M_PI / 180.0)

class Params : public vehicle_interfaces::GenericParams
{
public:
    std::vector<double> topicIds = { 0.0, 1.0, 2.0, 3.0 };
    std::string topicName = "ultrasound";
    float publishInterval_s = 0.03;
    std::string devicePath = "/dev/ttyACM0";
    int deviceBaud = 9600;
    float minRange = 0.2;
    float maxRange = 8.0;
    float fov = 15.0;
    float serialModuleProcTimeout_ms = 100.0;
    float msgUpdateProcTimeout_ms = 100.0;
    float msgPublishProcTimeout_ms = 200.0;

private:
    void _getParams()
    {
        this->get_parameter("topicIds", this->topicIds);
        this->get_parameter("topicName", this->topicName);
        this->get_parameter("publishInterval_s", this->publishInterval_s);
        this->get_parameter("devicePath", this->devicePath);
        this->get_parameter("deviceBaud", this->deviceBaud);
        this->get_parameter("minRange", this->minRange);
        this->get_parameter("maxRange", this->maxRange);
        this->get_parameter("fov", this->fov);
        this->get_parameter("serialModuleProcTimeout_ms", this->serialModuleProcTimeout_ms);
        this->get_parameter("msgUpdateProcTimeout_ms", this->msgUpdateProcTimeout_ms);
        this->get_parameter("msgPublishProcTimeout_ms", this->msgPublishProcTimeout_ms);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::vector<double> >("topicIds", this->topicIds);
        this->declare_parameter<std::string>("topicName", this->topicName);
        this->declare_parameter<float>("publishInterval_s", this->publishInterval_s);
        this->declare_parameter<std::string>("devicePath", this->devicePath);
        this->declare_parameter<int>("deviceBaud", this->deviceBaud);
        this->declare_parameter<float>("minRange", this->minRange);
        this->declare_parameter<float>("maxRange", this->maxRange);
        this->declare_parameter<float>("fov", this->fov);
        this->declare_parameter<float>("serialModuleProcTimeout_ms", this->serialModuleProcTimeout_ms);
        this->declare_parameter<float>("msgUpdateProcTimeout_ms", this->msgUpdateProcTimeout_ms);
        this->declare_parameter<float>("msgPublishProcTimeout_ms", this->msgPublishProcTimeout_ms);
        this->_getParams();
    }
};


class UltrasoundPublisher : public vehicle_interfaces::VehicleServiceNode
{
private:
    const std::shared_ptr<const Params> params_;
    std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, ULTRASOUND_MODULE_SIZE> pubs_;
    std::unique_ptr<vehicle_interfaces::LiteTimer> pubTm_;

    std::array<sensor_msgs::msg::Range, ULTRASOUND_MODULE_SIZE> msgs_;
    std::mutex msgMtx_;

private:
    void _pubTmCbFunc()
    {
        std::array<std::unique_ptr<sensor_msgs::msg::Range>, ULTRASOUND_MODULE_SIZE> msgs;
        {
            std::lock_guard<std::mutex> msgLock(this->msgMtx_);
            for (int i = 0; i < ULTRASOUND_MODULE_SIZE; i++)
            {
                msgs[i] = std::make_unique<sensor_msgs::msg::Range>();
                *msgs[i].get() = this->msgs_[i];
                msgs[i]->header.stamp = this->getTimestamp();
            }
        }

        for (int i = 0; i < ULTRASOUND_MODULE_SIZE; i++)
            this->pubs_[i]->publish(std::move(msgs[i]));

        this->updateProcedureMonitor("_pubTmCbFunc", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
    }

public:
    UltrasoundPublisher(const std::shared_ptr<const Params> params) : 
        vehicle_interfaces::VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        params_(params)
    {
        // Init msgs and publishers.
        std::vector<int> ids = { 0, 1, 2, 3 };
        if (ULTRASOUND_MODULE_SIZE != this->params_->topicIds.size())
        {
            RCLCPP_ERROR(this->get_logger(), "topicIds size must be %d", ULTRASOUND_MODULE_SIZE);
            RCLCPP_ERROR(this->get_logger(), "Use default topicIds...");
        }
        else
        {
            for (int i = 0; i < ULTRASOUND_MODULE_SIZE; i++)
                ids[i] = static_cast<int>(this->params_->topicIds[i]);
        }

        for (int i = 0; i < ULTRASOUND_MODULE_SIZE; i++)
        {
            std::string topicName = this->params_->topicName + "_" + std::to_string(ids[i]);
            this->msgs_[i] = sensor_msgs::msg::Range();
            this->msgs_[i].header.frame_id = topicName;
            this->msgs_[i].radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
            this->msgs_[i].field_of_view  = DEG2RAD(this->params_->fov);// ROS2 FoV in rad
            this->msgs_[i].min_range = this->params_->minRange;
            this->msgs_[i].max_range = this->params_->maxRange;
            this->msgs_[i].range = 0.0;

            this->pubs_[i] = this->create_publisher<sensor_msgs::msg::Range>(topicName, 10);
        }

        // Init publish timer.
        this->pubTm_ = std::make_unique<vehicle_interfaces::LiteTimer>(this->params_->publishInterval_s * 1000.0, std::bind(&UltrasoundPublisher::_pubTmCbFunc, this));
        this->pubTm_->start();

        this->addProcedureMonitor("_pubTmCbFunc", std::chrono::nanoseconds((int64_t)(params->msgPublishProcTimeout_ms * 1e6)));
        this->addProcedureMonitor("updateMsg", std::chrono::nanoseconds((int64_t)(params->msgUpdateProcTimeout_ms * 1e6)));
    }

    ~UltrasoundPublisher()
    {
        if (this->pubTm_)
            this->pubTm_->destroy();
    }

    void updateMsg(const std::array<float, ULTRASOUND_MODULE_SIZE>& range)
    {
        std::lock_guard<std::mutex> msgLock(this->msgMtx_);
        for (int i = 0; i < ULTRASOUND_MODULE_SIZE; i++)
            this->msgs_[i].range = range[i];
        this->updateProcedureMonitor("updateMsg", vehicle_interfaces::ProcedureStatus::PROCEDURE_STATUS_OK);
    }
};



#define FIRST_DATA_POS 4
#define DATA_SIZE 2
#define BUF_SIZE 16



int set_interface_attribs(int fd, int speed, int parity);
void set_blocking(int fd, int should_block);
bool ChkHeaderFooter(unsigned char *buffer);
int GetUltrasoundDataIter(unsigned char *buffer, int pos, int step);
std::vector<int> GetUltrasoundData(unsigned char *buffer);



class SerialModule
{
private:
    const std::string devicePath_;
    const int deviceBaud_;
    vehicle_interfaces::unique_thread th_;

    std::array<float, ULTRASOUND_MODULE_SIZE> msg_;
    std::chrono::time_point<std::chrono::system_clock> msgTs_;
    std::mutex msgMtx_;

    std::atomic<bool> exitF_;

private:
    void _th()
    {
        int fd = open(this->devicePath_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
        {
            std::cerr << "Error: Unable to open " << this->devicePath_.c_str() << std::endl;
            this->exitF_ = true;
            return;
        }

        if (this->deviceBaud_ == 9600)
            set_interface_attribs(fd, B9600, 0);    // set speed to 9600 bps, 8n1 (no parity)
        else if (this->deviceBaud_ == 19200)
            set_interface_attribs(fd, B19200, 0);   // set speed to 19200 bps, 8n1 (no parity)
        else if (this->deviceBaud_ == 38400)
            set_interface_attribs(fd, B38400, 0);   // set speed to 38400 bps, 8n1 (no parity)
        else if (this->deviceBaud_ == 57600)
            set_interface_attribs(fd, B57600, 0);   // set speed to 57600 bps, 8n1 (no parity)
        else if (this->deviceBaud_ == 115200)
            set_interface_attribs(fd, B115200, 0);  // set speed to 115200 bps, 8n1 (no parity)
        else
            set_interface_attribs(fd, B9600, 0);    // set speed to 9600 bps, 8n1 (no parity)

        // set_blocking(fd, 0);                        // set no blocking


        unsigned char buffer[BUF_SIZE];
        int errCnt = 0;
        while (!this->exitF_.load() && errCnt < 10)
        {
            int n = read(fd, buffer, sizeof(buffer));// Blocked until receives BUF_SIZE bytes
            if (n > 0)
            {
                if (ChkHeaderFooter(buffer))
                {
                    std::vector<int> data = GetUltrasoundData(buffer);

                    std::lock_guard<std::mutex> msgLock(this->msgMtx_);
                    this->msg_ = { static_cast<float>(data[0]), static_cast<float>(data[1]), static_cast<float>(data[2]), static_cast<float>(data[3]) };
                    this->msgTs_ = std::chrono::system_clock::now();
                    errCnt = 0;
                }
                else
                {
                    std::cout << "Invalid data" << std::endl;
                    errCnt++;
                }
            }
        }

        if (fd > 0)
        {
            write(fd, "COCOGGBIRD", 10);
            close(fd);
        }

        this->exitF_ = true;
    }

public:
    SerialModule(std::string devicePath, int baud) : 
        devicePath_(devicePath), 
        deviceBaud_(baud), 
        exitF_(false)
    {
        this->msg_ = { 0.0, 0.0, 0.0, 0.0 };
        this->msgTs_ = std::chrono::system_clock::now();
        this->th_ = vehicle_interfaces::make_unique_thread(&SerialModule::_th, this);
    }

    ~SerialModule()
    {
        this->exitF_ = true;
        this->th_.reset();
    }

    bool getMsg(std::array<float, ULTRASOUND_MODULE_SIZE>& outMsg, std::chrono::time_point<std::chrono::system_clock>& outTs)
    {
        if (this->exitF_.load())
            return false;
        std::lock_guard<std::mutex> msgLock(this->msgMtx_);
        outMsg = this->msg_;
        outTs = this->msgTs_;
        return true;
    }

    bool isExit() const
    {
        return this->exitF_.load();
    }
};



// Ref: https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
int set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    if (tcgetattr (fd, &tty) != 0)
    {
        std::cerr << "error " << errno << " from tcgetattr" << std::endl;
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "error " << errno << " from tcsetattr" << std::endl;
        return -1;
    }
    return 0;
}



void set_blocking(int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        std::cerr << "error " << errno << " from tggetattr" << std::endl;
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        std::cerr << "error " << errno << " setting term attributes" << std::endl;
}



bool ChkHeaderFooter(unsigned char *buffer)
{
    return buffer[0] == 'C' && buffer[1] == 'O' && buffer[2] == 'C' && buffer[3] == 'O' && 
              buffer[12] == 'B' && buffer[13] == 'I' && buffer[14] == 'R' && buffer[15] == 'D';
}



int GetUltrasoundDataIter(unsigned char *buffer, int pos, int step)
{
    if (step == 0) return 0;
    return static_cast<int>(buffer[pos]) << 8 * (step - 1) | GetUltrasoundDataIter(buffer, pos + 1, step - 1);
}



std::vector<int> GetUltrasoundData(unsigned char *buffer)
{
    std::vector<int> ret;
    ret.reserve(ULTRASOUND_MODULE_SIZE);

    for (int i = FIRST_DATA_POS; i < FIRST_DATA_POS + ULTRASOUND_MODULE_SIZE * DATA_SIZE; i += DATA_SIZE)
    {
        // int data = (buffer[i] << 8) | buffer[i + 1];
        ret.push_back(GetUltrasoundDataIter(buffer, i, DATA_SIZE));
    }
    return ret;
}