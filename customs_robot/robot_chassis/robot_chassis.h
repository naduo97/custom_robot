#ifndef ROBOTCHASSIS_H
#define ROBOTCHASSIS_H
#include "work_base.h"
#include <functional>
#include <tcp_client.h>
#include "ndc8_tcp_client.h"
#include "nmc_tcp_client.h"
#include "chassis_state_data.h"
#include <common/common.h>
#include <string>
#include <sstream>
#include <fstream>
#include <unordered_map>

#include "LLog/LLog.h"

namespace RobotChassis{

typedef std::function<void(const ChassisStateData&)> STATE_CALLBACK;

class RobotChassis : public CustomsRobot::WorkBase
{
public:
    RobotChassis(STATE_CALLBACK call_back);
    ~RobotChassis();

public:
    bool SendGoal(const Common::Pose &pose);
    bool SendGoal(int id); // todo
    void CalculateOffsetCoordinate(double angle, double distance, double& x, double& y);
    double ConvertAngleChassisToTrueNorth(double angle);    // 底盘坐标 转到 正北坐标
    double ConvertAngleTrueNorthToChassis(double angle);    // 正北坐标 转到 底盘坐标
    int QuantitativeMove(double angle, double distance, double horizontal_offset);   // 定量移动 角度，距离
    void QuantitativeAdvance(double distance, double horizontal_offset);
    void QuantitativeReceding(double distance, double horizontal_offset);
    void WorkFun();
    void ControlData(const ChassisControl& control_data);
    void NDC8_CallBack(const std::string &attr_name, const std::string &attr_value, const std::string &value);
    void NMC_CallBack(const std::string &data);
    void CalculateRelativePosition(double angle, double length, double& x, double& y);
    void TcpClientCallBack(const ChassisStateData& data);

    void Pause();   // 暂停

    // ----- 控制响应函数 ------
    // 前进
    bool ControlForward();
    // 后退
    bool ControlBackwards();
    // 向左
    bool ControlLeft();
    // 向右
    bool ControlRight();
    // 停止
    bool ControlStop();
    // 旋转
    bool ControlTurn(double angle);

    // 获取雷达
    bool GetLidar(int id);

public:
    ChassisStateData ChassisStateData_;
private:
    LLOG::LLog Log_;
    bool bMoving_;  // 记录当前底盘正在移动中
    bool bExit_;    // 是否退出
    int PauseBackupTarget_; // 暂停备份需要 前往的目标点
    NavigationState PauseBackupState_; // 暂停备份 暂停前的点
    NDC8TcpClient NDC8TcpClient_;   //  主要用于获取底盘状态
    NMCTcpClient NMCTcpClient_;     // 主要用于获取雷达
    long long llSendTime_;          // 用于计算时间
    STATE_CALLBACK pStateCallback_;
    ChassisControlVelocity ControlVelocity_;
    std::unordered_map<int, Common::Pose> umapStationsPoint_;   // 记录站点
};
}
#endif // ROBOTCHASSIS_H
