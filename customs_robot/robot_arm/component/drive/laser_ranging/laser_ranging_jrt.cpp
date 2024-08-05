#include "laser_ranging_jrt.h"

namespace RobotArm{

LaserRangingJrt::LaserRangingJrt(std::string device_name, int baud_rate)
    : Device_(device_name, baud_rate)
{
    bStop_ = false;
    thUpdata_ = std::thread(&LaserRangingJrt::ThreadFun, this);
}

LaserRangingJrt::~LaserRangingJrt()
{

}

void LaserRangingJrt::ThreadFun()
{
    while(!bStop_)
    {
        if (!Device_.IsCommunication())
        {
            if (Device_.EstablishCommunication())
            {
                // 连接失败，过一会重连
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }
        // 已经连接成功
        std::string get_data;
        Device_.ReadData(get_data);
        strBufferData_ += get_data;
        while (strBufferData_.size() > 4)   // 需要识别消息类型标签 所以至少4个字节
        {
            int head_index = strBufferData_.find(0xAA);
//            int end_index = strBufferData_.find("**\r", head_index);
            // 距离返回消息
            if (head_index > -1
                && strBufferData_[head_index+2] == 0x2
                && strBufferData_[head_index+3] == 0x22)
            {   // 判断长度对不对
                if((int)strBufferData_.size() < head_index + 13) // 该消息有13个字节
                {
                    break;
                }
                //  todo 需要考虑 多个设备串联的问题

            }
            else
            {
                break;
            }
        }
    }
}

}
