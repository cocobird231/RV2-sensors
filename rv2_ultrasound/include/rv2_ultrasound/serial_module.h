#pragma once
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <rv2_interfaces/utils.h>
#include <rv2_ultrasound/config.h>

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
    const std::string mDevicePath_;
    const int mDeviceBaud_;
    rv2_interfaces::unique_thread mTh_;

    std::array<float, ULTRASOUND_MODULE_SIZE> mMsg_;
    std::chrono::time_point<std::chrono::system_clock> mMsgTs_;
    std::mutex mMsgMtx_;

    std::atomic<bool> mExitF_;

private:
    void _th();

public:
    SerialModule(std::string devicePath, int baud);

    ~SerialModule();

    bool getMsg(std::array<float, ULTRASOUND_MODULE_SIZE>& outMsg, std::chrono::time_point<std::chrono::system_clock>& outTs);

    bool isExit() const;
};

