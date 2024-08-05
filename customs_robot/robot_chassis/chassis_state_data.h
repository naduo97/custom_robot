#ifndef CHASSISSTATEDATA_H
#define CHASSISSTATEDATA_H
#include <common/common.h>
#include <string>
#include <opencv2/opencv.hpp>

namespace RobotChassis{

enum ChassisState
{
    Uninitialized = 0,  //  未初始化
    NetworkNormal = 1,  //  通讯正常
    NetworkAnomaly = 2, //  通讯异常
    ChassisError = 3    //  底盘错误
};

enum NavigationState
{
    Idie = 0,
    Underway = 1,
    Arrive = 2,
    Pause = 3
};


class ChassisStateData
{
public:
    ChassisStateData();
    ~ChassisStateData();

public:
    ChassisState ChassisState_;         // 底盘状态 管理通讯是否正常之类的
    NavigationState NavigationState_;   // 导航状态 管理是否到达之类的
    cv::Mat matLidarImage_;     // 雷达点云图
    Common::Pose Pose_;         // 底盘当前位置 和 姿态
    Common::Pose TargetPose_;   // 底盘目标位置 和 姿态
    double  dVoltage_;          //底盘电压
    int iCurrentStationID_;     //当前站点ID
    int iTargetStationID_;     // 目标站点ID
};

// -----------------------------

class ChassisControl
{
public:
    enum ControlType
    {
        Init = 0,           // 用于初始化
        NMC_Init = 1,       // 用于NMC初始化
        NDC8_Init = 2,       // 用于NDC8初始化
        On_Forward = 3,
        On_Backwards = 4,
        On_Left = 5,
        On_Right = 6,
        On_Stop = 7,
        On_Turn = 8,
        Get_Lidar = 9,
        Send_Goal = 10
    };
public:
    ChassisControl();
    ~ChassisControl();

public:
    ControlType ControlType_;
    std::string strValue_;
    int iValue_;
    double dValue_;
};


// --------------------------------
//  底盘速度数据
class ChassisControlVelocity
{
public:
    ChassisControlVelocity()
        :dLinearvelocity_(0.0)
        ,dAngleSpeed_(0.0)
        ,dAngle_(0.0)
    {}
    ~ChassisControlVelocity(){}

    void SetZero()
    {
        dLinearvelocity_ = 0.0;
        dAngleSpeed_ = 0.0;
        dAngle_ = 0.0;
    }

public:
    double dLinearvelocity_;    // 线性速度
    double dAngleSpeed_;    // 角速度
    double dAngle_;         //角度
};

}
#endif // CHASSISSTATEDATA_H
