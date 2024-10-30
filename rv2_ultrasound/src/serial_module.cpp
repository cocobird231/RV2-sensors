// #include <rv2_ultrasound/serial_module.h>
#include "../include/rv2_ultrasound/serial_module.h"// TEST

namespace rv2_sensors
{

SerialModule::SerialModule(std::string devicePath, int baud) : 
    mDevicePath_(devicePath), 
    mDeviceBaud_(baud), 
    mExitF_(false)
{
    mMsg_ = { 0.0, 0.0, 0.0, 0.0 };
    mMsgTs_ = std::chrono::system_clock::now();
    mTh_ = rv2_interfaces::make_unique_thread(&SerialModule::_th, this);
}



SerialModule::~SerialModule()
{
    mExitF_ = true;
    mTh_.reset();
}



void SerialModule::_th()
{
    int fd = open(mDevicePath_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        std::cerr << "Error: Unable to open " << mDevicePath_.c_str() << std::endl;
        mExitF_ = true;
        return;
    }

    if (mDeviceBaud_ == 9600)
        set_interface_attribs(fd, B9600, 0);    // set speed to 9600 bps, 8n1 (no parity)
    else if (mDeviceBaud_ == 19200)
        set_interface_attribs(fd, B19200, 0);   // set speed to 19200 bps, 8n1 (no parity)
    else if (mDeviceBaud_ == 38400)
        set_interface_attribs(fd, B38400, 0);   // set speed to 38400 bps, 8n1 (no parity)
    else if (mDeviceBaud_ == 57600)
        set_interface_attribs(fd, B57600, 0);   // set speed to 57600 bps, 8n1 (no parity)
    else if (mDeviceBaud_ == 115200)
        set_interface_attribs(fd, B115200, 0);  // set speed to 115200 bps, 8n1 (no parity)
    else
        set_interface_attribs(fd, B9600, 0);    // set speed to 9600 bps, 8n1 (no parity)

    // set_blocking(fd, 0);                        // set no blocking


    unsigned char buffer[BUF_SIZE];
    int errCnt = 0;
    while (!mExitF_.load() && errCnt < 10)
    {
        int n = read(fd, buffer, sizeof(buffer));// Blocked until receives BUF_SIZE bytes
        if (n > 0)
        {
            if (ChkHeaderFooter(buffer))
            {
                std::vector<int> data = GetUltrasoundData(buffer);

                std::lock_guard<std::mutex> msgLock(mMsgMtx_);
                mMsg_ = { static_cast<float>(data[0]), static_cast<float>(data[1]), static_cast<float>(data[2]), static_cast<float>(data[3]) };
                mMsgTs_ = std::chrono::system_clock::now();
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

    mExitF_ = true;
}



bool SerialModule::getMsg(std::array<float, ULTRASOUND_MODULE_SIZE>& outMsg, std::chrono::time_point<std::chrono::system_clock>& outTs)
{
    if (mExitF_.load())
        return false;
    std::lock_guard<std::mutex> msgLock(mMsgMtx_);
    outMsg = mMsg_;
    outTs = mMsgTs_;
    return true;
}



bool SerialModule::isExit() const
{
    return mExitF_.load();
}



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

} // namespace rv2_sensors
