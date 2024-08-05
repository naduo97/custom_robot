#include "ranging_laser_long.h"

namespace RobotArm{
RangingLaser_Long::RangingLaser_Long(std::string device_name, int baud_rate)
    :CustomsRobot::WorkBase(CustomsRobot::WhileType::TIME)
    ,Device_(device_name, baud_rate)
{
    this->SetSleep(30);
    this->ExecutionThread();

    bThreadStop_ = false;
    dDistance_ = 0.0;
    thGetDataThread_ = std::thread(&RangingLaser_Long::DataThread, this);
}

RangingLaser_Long::~RangingLaser_Long()
{
    bThreadStop_ = true;
    Device_.StopCommunication();
    CustomsRobot::WorkBase::Stop();
    if (thGetDataThread_.joinable())
    {
        thGetDataThread_.join();
    }
}

double RangingLaser_Long::GetDistance()
{
    return dDistance_;
}


void RangingLaser_Long::DataThread()
{
    while (!bThreadStop_)
    {
        if (!Device_.IsCommunication())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        std::string get_data;
        Device_.ReadData(get_data);
        if (get_data.size() == 0)
        {
            continue;
        }
        strBufferData_ += get_data;
//        5F 03 02 05 6C 12 F4
        while (1)
        {
            int head_index = strBufferData_.find(0x5F);
            if ((head_index > -1) && ((head_index + 7) <= (int)strBufferData_.size()))
            {
                uint16_t iDist = (uint16_t)(((strBufferData_[head_index + 3] & 0xFF) << 8) | (strBufferData_[head_index + 4] & 0xFF));
                dDistance_ = (double)iDist;
                strBufferData_ = strBufferData_.substr(head_index + 7);
            }
            else
            {
                break;
            }
        }
    }
}


void RangingLaser_Long::WorkFun()
{
    if (!Device_.IsCommunication())
    {
        Device_.EstablishCommunication();
    }
    // 5F 03 00 20 00 01 88 BE
    char send_data[] = {static_cast<char>(0x5f), static_cast<char>(0x03), static_cast<char>(0x00)
                        , static_cast<char>(0x20), static_cast<char>(0x00), static_cast<char>(0x01)
                        , static_cast<char>(0x88), static_cast<char>(0xbe)};
    Device_.WriteData(std::string(send_data, 8));
}

}
