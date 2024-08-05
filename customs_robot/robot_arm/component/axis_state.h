#ifndef AXISSTATE_H
#define AXISSTATE_H
#include "robot_arm/zmcaux.h"
#include "robot_arm/zmotion.h"

namespace RobotArm
{

class AxisState
{
public:
    AxisState(int ID);
    AxisState();
    ~AxisState();
    bool Update(ZMC_HANDLE handle);      // 更新数据

    // ---------------设置---------------
    bool SetType(ZMC_HANDLE handle, int iValue);
    bool SetEnable(ZMC_HANDLE handle, int iValue);
    bool SetUnits(ZMC_HANDLE handle, float fValue);    // 设置 脉冲当量
    bool SetLspeed(ZMC_HANDLE handle, float fValue);   // 设置 起始速度
    bool SetAccel(ZMC_HANDLE handle, float fValue);    // 设置 加速度
    bool SetDecel(ZMC_HANDLE handle, float fValue);    // 设置 减速度
    bool SetSpeed(ZMC_HANDLE handle, float fValue);    // 设置 运行速度
    bool SetCreep(ZMC_HANDLE handle, float fValue);    // 设置 归零时的爬行速度
    bool SetMerge(ZMC_HANDLE handle, float fValue);    // 设置 连续插补开关
    bool SetForceSpeed(ZMC_HANDLE handle, float fValue);    // 设置 SP 的运行速度。
    bool SetStartMoveSpeed(ZMC_HANDLE handle, float fValue);    // 设置 自定义速度的 SP 运动的起始速度。
    bool SetEndMoveSpeed(ZMC_HANDLE handle, float fValue);      // 设置 自定义速度的 SP 运动的结束速度。
    bool SetDatumIn(ZMC_HANDLE handle, int iValue);     // 设置 回零原点信号
    bool SetDpos(ZMC_HANDLE handle, float fValue);      // 设置 Dpos
    bool SetMpos(ZMC_HANDLE handle, float fValue);      // 设置 Mpos
    // ---------------获取---------------
    bool GetRemainBuffer(ZMC_HANDLE handle, int* piValue);  // 获取 轴当前剩余的缓存空间大小
    // ---------------功能---------------
    bool Stop(ZMC_HANDLE handle);   // 停止运动
    bool ContinueRun(ZMC_HANDLE handle, int idir);          // 持续运行
    bool SingleMoveSp(ZMC_HANDLE handle, float fDisance);   // 带加减速绝对运动运动
    bool SingleDatum(ZMC_HANDLE handle, int iMode);  // 单轴回零运动
    bool MoveDelay(ZMC_HANDLE handle, int iTime);   //缓冲区中加入延时指令


public:
    int iAxisID_;   // 轴ID
    float fUnits_;  // 脉冲当量
    float fAccel_;  // 加速度
    float fDecel_;  // 减速度
    float fSpeed_;  // 速度
};


}
#endif // AXISSTATE_H
