#include "ranging_laser.h"
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>

namespace RobotArm{
RangingLaser::RangingLaser(std::string device_name, int baud_rate, int id)
    :CustomsRobot::WorkBase(CustomsRobot::WhileType::TIME)
    ,Device_(device_name, baud_rate)
{
    this->SetSleep(30);
    this->ExecutionThread();

    std::stringstream str_id;
    str_id << std::setw(2) << std::setfill('0') << id;
    strID_ = str_id.str().c_str();
    iID_ = id;
    bThreadStop_ = false;
    dDistance_ = 0.0;
    thGetDataThread_ = std::thread(&RangingLaser::DataThread, this);
}

RangingLaser::~RangingLaser()
{
    bThreadStop_ = true;
    Device_.StopCommunication();
    CustomsRobot::WorkBase::Stop();
    if (thGetDataThread_.joinable())
    {
        thGetDataThread_.join();
    }
}

double RangingLaser::GetDistance()
{
    return dDistance_;
}

int RangingLaser::GetID()
{
    return iID_;
}

void RangingLaser::DataThread()
{
    while (!bThreadStop_)
    {
        if (!Device_.IsCommunication())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }
        std::string get_data;
        Device_.ReadData(get_data);
        if (get_data.size() == 0)
        {
            continue;
        }
        strBufferData_ += get_data;
        while (1)
        {
            int head_index = strBufferData_.find("%");
            int end_index = strBufferData_.find("**\r");
            if (head_index > -1 && end_index > -1)
            {
                if (strBufferData_.substr(head_index + 4, 3) == "RMD")
                {
                    double temp_data = 0.0;
                    char *temp_ptr = nullptr;
                    temp_data = std::strtod(strBufferData_.substr(head_index+7, 8).c_str(), &temp_ptr);
                    dDistance_ = temp_data / 10000;  //  + 310;
                }
                strBufferData_ = strBufferData_.substr(end_index + 3);
            }
            else
            {
                break;
            }
        }
    }
}

void RangingLaser::WorkFun()
{
    if (!Device_.IsCommunication())
    {
        Device_.EstablishCommunication();
    }
    std::string send_data;
    send_data += 0x25;
    send_data += strID_;
    send_data += 0x23;
    send_data += 0x52;
    send_data += 0x4D;
    send_data += 0x44;
    send_data += 0x2a;
    send_data += 0x2a;
    send_data += 0x0D;
    Device_.WriteData(send_data);
}
}
