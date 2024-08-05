#ifndef RANGINGLASER_H
#define RANGINGLASER_H
#include <thread>
#include "work_base.h"
#include "serial_communication.h"

namespace RobotArm{
class RangingLaser : public CustomsRobot::WorkBase
{
public:
    RangingLaser(std::string device_name, int baud_rate, int id);
    ~RangingLaser();

    double GetDistance();
    int GetID();

    void DataThread();
    void WorkFun();

private:
    std::thread thGetDataThread_;
    bool bThreadStop_;
    CustomsRobot::SerialCommunication Device_;
    std::string strID_;
    int iID_;
    double dDistance_;
    std::string strBufferData_;
};
}
#endif // RANGINGLASER_H
