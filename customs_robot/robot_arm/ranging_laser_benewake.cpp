#include "ranging_laser_benewake.h"
#include <iostream>

namespace RobotArm{

RangingLaser_Benewake::RangingLaser_Benewake(std::string device_name, int baud_rate)
    :CustomsRobot::WorkBase(CustomsRobot::WhileType::TIME)
    ,Device_(device_name, baud_rate)
{
    dDistance_ = 0.0;
    this->SetSleep(50);
    this->ExecutionThread();
}

RangingLaser_Benewake::~RangingLaser_Benewake()
{
    Device_.StopCommunication();
}


double RangingLaser_Benewake::GetDistance()
{
    return dDistance_;
}

void RangingLaser_Benewake::WorkFun()
{
    if (!Device_.IsCommunication())
    {
        Device_.EstablishCommunication();
    }
    else
    {
        std::string data;
        Device_.ReadData(data);
        if (data.size() == 0)
        {
            return;
        }
        strBufferData_ += data;
        // 59 59 13 00 24 15 70 09 77
        // 59 59 59 00 15 0c 90 09 c5
        while (1)
        {
            int head_index = strBufferData_.find("YY"); // 0x59 0x59
            if ((head_index > -1) && (head_index + 9 <= (int)strBufferData_.size()))
            {
                uint16_t iDist = (uint16_t)(((strBufferData_[head_index + 3] & 0xFF) << 8) | (strBufferData_[head_index + 2] & 0xFF));
                dDistance_ = (double)iDist;
                strBufferData_ = strBufferData_.substr(head_index + 9);
            }
            else
            {
                break;
            }
        }
    }
}

}
