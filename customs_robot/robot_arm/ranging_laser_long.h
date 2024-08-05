#ifndef RANGINGLASER_LONG_H
#define RANGINGLASER_LONG_H
#include <thread>
#include "work_base.h"
#include "serial_communication.h"

namespace RobotArm{
class RangingLaser_Long : public CustomsRobot::WorkBase
{
public:
    RangingLaser_Long(std::string device_name, int baud_rate);
    ~RangingLaser_Long();

    double GetDistance();

    void DataThread();
    void WorkFun();

private:
    std::thread thGetDataThread_;
    bool bThreadStop_;
    CustomsRobot::SerialCommunication Device_;
//    std::string strID_;
//    int iID_;
    double dDistance_;
    std::string strBufferData_;
};
}
#endif // RANGINGLASER_LONG_H
