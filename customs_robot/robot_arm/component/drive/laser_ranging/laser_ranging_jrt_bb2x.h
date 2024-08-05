#ifndef LASERRANGINGJRTBB2X_H
#define LASERRANGINGJRTBB2X_H
#include <thread>
#include "work_base.h"
#include "serial_communication.h"


namespace RobotArm{
class LaserRangingJrtBb2x
{
public:
    LaserRangingJrtBb2x(std::string device_name, int baud_rate);
    ~LaserRangingJrtBb2x();

public:
    bool OpenDevice();  // 开启设备
    bool CloseDevice();  // 关闭设备
    bool StopMeasurement(); // 停止测量
    bool GetAnAutoMeasurement();    //获取一次自动测量
    bool GetAnSlowMeasurement();    // 慢速获取一次
    bool GetAnFastMeasurement();    // 快速获取一次
    bool ContinuousAcquisitionAutoMeasurement();    //连续获取自动测量
    bool ContinuousAcquisitionSlowMeasurement();    //连续获取慢速测量
    bool ContinuousAcquisitionFastMeasurement();    //连续获取快速测量

    uint32_t GetDistance();
private:
    void ThreadFun();
    std::string ErrorValue2String(uint16_t error_value);

private:
    CustomsRobot::SerialCommunication Device_;
    std::string strBufferData_;
    uint32_t uiDistance_;
    uint16_t uiLastError_;
    bool bStop_;
    long long llLastDataTime_;
    std::thread thUpdata_;
};
}
#endif // LASERRANGINGJRTBB2X_H
