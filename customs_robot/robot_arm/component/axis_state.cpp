#include "axis_state.h"
#include <QDebug>
namespace RobotArm
{
AxisState::AxisState(int ID)
{
    iAxisID_ = ID;
}

AxisState::AxisState()
{
    iAxisID_ = 0;
}

AxisState::~AxisState()
{

}

bool AxisState::SetType(ZMC_HANDLE handle, int iValue)
{
    return ZAux_Direct_SetAtype(handle, iAxisID_, iValue) == 0 ? true : false;
}

bool AxisState::SetEnable(ZMC_HANDLE handle, int iValue)
{
    return ZAux_Direct_SetAxisEnable(handle, iAxisID_, iValue) == 0 ? true : false;
}

// 设置脉冲当量
bool AxisState::SetUnits(ZMC_HANDLE handle, float fValue)
{
    fUnits_ = fValue;
    return ZAux_Direct_SetUnits(handle, iAxisID_, fValue) == 0 ? true : false;
}

// 设置 起始速度
bool AxisState::SetLspeed(ZMC_HANDLE handle, float fValue)
{
    return ZAux_Direct_SetLspeed(handle, iAxisID_, fValue) == 0 ? true : false;
}

// 设置加速度
bool AxisState::SetAccel(ZMC_HANDLE handle, float fValue)
{
    fAccel_ = fValue;
    return ZAux_Direct_SetAccel(handle, iAxisID_, fValue) == 0 ? true : false;
}

// 设置减速度
bool AxisState::SetDecel(ZMC_HANDLE handle, float fValue)
{
    fDecel_ = fValue;
    return ZAux_Direct_SetDecel(handle, iAxisID_, fValue) == 0 ? true : false;
}

// 设置运行速度
bool AxisState::SetSpeed(ZMC_HANDLE handle, float fValue)
{
    if (fValue != fSpeed_)
    {

        SetAccel(handle, fValue * 5);
        SetDecel(handle, fValue * 5);
        fSpeed_ = fValue;
    }
    else if (fValue == 0.0)
    {
        SetDecel(handle, fSpeed_);
        fSpeed_ = fValue;
    }
    return ZAux_Direct_SetSpeed(handle, iAxisID_, fValue) == 0 ? true : false;
}

bool AxisState::SetCreep(ZMC_HANDLE handle, float fValue)
{
    return ZAux_Direct_SetCreep(handle, iAxisID_, fValue) == 0 ? true : false;
}

// 设置 连续插补开关
bool AxisState::SetMerge(ZMC_HANDLE handle, float fValue)
{
    return ZAux_Direct_SetMerge(handle, iAxisID_, fValue) == 0 ? true : false;
}

// 设置 SP 的运行速度。
bool AxisState::SetForceSpeed(ZMC_HANDLE handle, float fValue)
{
    return ZAux_Direct_SetForceSpeed(handle, iAxisID_, fValue) == 0 ? true : false;
}

// 设置 自定义速度的 SP 运动的起始速度。
bool AxisState::SetStartMoveSpeed(ZMC_HANDLE handle, float fValue)
{
    return ZAux_Direct_SetStartMoveSpeed(handle, iAxisID_, fValue) == 0 ? true : false;
}

// 设置 自定义速度的 SP 运动的结束速度。
bool AxisState::SetEndMoveSpeed(ZMC_HANDLE handle, float fValue)
{
    return ZAux_Direct_SetEndMoveSpeed(handle, iAxisID_, fValue) == 0 ? true : false;
}

// 设置 回零原点信号
bool AxisState::SetDatumIn(ZMC_HANDLE handle, int iValue)
{
    return ZAux_Direct_SetDatumIn(handle, iAxisID_, iValue) == 0 ? true : false;
}

// 设置 Dpos
bool AxisState::SetDpos(ZMC_HANDLE handle, float fValue)
{
    return ZAux_Direct_SetDpos(handle, iAxisID_, fValue) == 0 ? true : false;
}

// 设置 Mpos
bool AxisState::SetMpos(ZMC_HANDLE handle, float fValue)
{
    return ZAux_Direct_SetMpos(handle, iAxisID_, fValue) == 0 ? true : false;
}

// 获取 轴当前剩余的缓存空间大小
bool AxisState::GetRemainBuffer(ZMC_HANDLE handle, int* piValue)
{
    return ZAux_Direct_GetRemain_Buffer(handle, iAxisID_, piValue) == 0 ? true : false;
}


bool AxisState::ContinueRun(ZMC_HANDLE handle, int idir)
{
    return ZAux_Direct_Single_Vmove(handle, iAxisID_, idir) == 0 ? true : false;
}

// 带加减速绝对运动运动
bool AxisState::SingleMoveSp(ZMC_HANDLE handle, float fDisance)
{
    int axis_id = iAxisID_;
    float dpos = fDisance;
    return ZAux_Direct_MoveSp(handle, 1, &axis_id, &dpos) == 0 ? true : false;
}

// 单轴回零运动
bool AxisState::SingleDatum(ZMC_HANDLE handle, int iMode)
{
    return ZAux_Direct_Singl_Datum(handle, iAxisID_, iMode) == 0 ? true : false;
}

//缓冲区中加入延时指令
bool AxisState::MoveDelay(ZMC_HANDLE handle, int iTime)
{
    return ZAux_Direct_MoveDelay(handle, iAxisID_, iTime) == 0 ? true : false;
}

// 更新数据
bool AxisState::Update(ZMC_HANDLE handle)
{
    if (handle == nullptr)
    {
        return false;
    }
    if ((ZAux_Direct_GetUnits(handle, iAxisID_, &fUnits_) != 0)
       || (ZAux_Direct_GetAccel(handle, iAxisID_, &fAccel_) != 0)
       || (ZAux_Direct_GetDecel(handle, iAxisID_, &fDecel_) != 0)
       || (ZAux_Direct_GetSpeed(handle, iAxisID_, &fSpeed_) != 0))
    {
        return false;
    }
    return true;
}

// 传送带停止运动
bool AxisState::Stop(ZMC_HANDLE handle)
{
    return ZAux_Direct_Single_Cancel(handle, iAxisID_, 2) == 0 ? true : false;
}


}
