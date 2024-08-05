#ifndef LASERRANGINGJRT_H
#define LASERRANGINGJRT_H

#include <thread>
#include "work_base.h"
#include "serial_communication.h"


namespace RobotArm{
class LaserRangingJrt
{
public:
    LaserRangingJrt(std::string device_name, int baud_rate);
    ~LaserRangingJrt();

public:

    // 慢速获取一次
    // 快速获取一次
    //
private:
    void ThreadFun();

private:
    CustomsRobot::SerialCommunication Device_;
    std::string strBufferData_;
    double dDistance_;
    bool bStop_;
    std::thread thUpdata_;
};
}
#endif // LASERRANGINGJRT_H
