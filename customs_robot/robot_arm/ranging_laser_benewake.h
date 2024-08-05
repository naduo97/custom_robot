#ifndef RANGINGLASER_BENEWAKE_H
#define RANGINGLASER_BENEWAKE_H
#include "work_base.h"
#include "serial_communication.h"

namespace RobotArm{
class RangingLaser_Benewake : public CustomsRobot::WorkBase
{
public:
    RangingLaser_Benewake(std::string device_name, int baud_rate);
    ~RangingLaser_Benewake();
    double GetDistance();
    void WorkFun();
private:
//    std::thread thGetDataThread_;
//    bool bThreadStop_;
    CustomsRobot::SerialCommunication Device_;
    double dDistance_;
    std::string strBufferData_;
};
}

#endif // RANGINGLASER_BENEWAKE_H
