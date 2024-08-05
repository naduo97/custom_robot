#include "robot_arm.h"
#include <QtCore>
#include <QMessageBox>
#include <QCoreApplication>
#include <string>
#include <sstream>
#include <termios.h>
#include <cstdio>
#include <array>
#include <algorithm>

namespace RobotArm{


RobotArm::RobotArm(STATE_CALLBACK call_back)
    : CustomsRobot::WorkBase(CustomsRobot::TIME),
      log_(LLOG::TLogLevel::DEBUG, "RobotArm", "./log", 200),
      FrontLaserDist_("/dev/ttyUSB0", B115200)
{
    bRepeatFunction_ = false;
    llGraspWaitTime_ = 0;
    bTryGrasp_ = false;
    ZMC_Handle_ = nullptr;
    bRobotArmResume_ = false;
    bManualMoveDone_ = false;
    pStateCallback_ = call_back;
    WorkBase::SetSleep(100);
    WorkBase::ExecutionThread();
    RobotArmStateData_.dLeftLimitPoint_ = 850;//775;
    RobotArmStateData_.dRightLimitPoint_ = -900;//-885;
    RobotArmStateData_.RobotArmState_ = RobotArmState::NetworkAnomaly;
    RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;

    RobotArmStateData_.fLeftEdgeReturnAngle_ = 10.0;
    RobotArmStateData_.fLeftEdgeReturnDis_ = 145.0;
    RobotArmStateData_.fLeftEdgeReturnXDis_ = 10.0;
    RobotArmStateData_.fRightEdgeReturnAngle_ = -10.0;
    RobotArmStateData_.fRightEdgeReturnDis_ = 145.0;
    RobotArmStateData_.fRightEdgeReturnXDis_ = 10.0;

    pStateCallback_(RobotArmStateData_);
    memset(ip_, 0, 16);
    memcpy(ip_,"192.168.0.11", 12);
    ZMC_Handle_ = nullptr;
    bIsStopUpdate_ = true;
    bRobotArmPause_ = false;
    bIsPushedOut_ = false;

    bTestModeEnablementConveyor_ = false;
    // 机械臂轴状态初始化
    iMode_12_ = 0;
    iMode_14_ = 0;
    iTuboMode_ = 0;

    RobotArmStateData_.umapAxisStatus_[0] = 0;
    RobotArmStateData_.umapAxisStatus_[1] = 0;
    RobotArmStateData_.umapAxisStatus_[2] = 0;
    RobotArmStateData_.umapAxisStatus_[3] = 0;
    RobotArmStateData_.umapAxisStatus_[4] = 0;
    RobotArmStateData_.umapAxisStatus_[5] = 0;
    RobotArmStateData_.umapAxisStatus_[6] = 0;
    RobotArmStateData_.umapAxisStatus_[7] = 0;
    RobotArmStateData_.umapAxisStatus_[8] = 0;
    RobotArmStateData_.umapAxisStatus_[9] = 0;
    RobotArmStateData_.umapAxisStatus_[10] = 0;
    RobotArmStateData_.umapAxisStatus_[11] = 0;
    RobotArmStateData_.umapAxisStatus_[12] = 0;
    RobotArmStateData_.umapAxisStatus_[13] = 0; // 推杆轴
    RobotArmStateData_.umapAxisStatus_[14] = 0; // 大臂传送带轴
    RobotArmStateData_.umapAxisStatus_[15] = 0; // 大臂传送带轴

    RobotArmStateData_.umapAxisStatusString_[0] = "";
    RobotArmStateData_.umapAxisStatusString_[1] = "";
    RobotArmStateData_.umapAxisStatusString_[2] = "";
    RobotArmStateData_.umapAxisStatusString_[3] = "";
    RobotArmStateData_.umapAxisStatusString_[4] = "";
    RobotArmStateData_.umapAxisStatusString_[5] = "";
    RobotArmStateData_.umapAxisStatusString_[6] = "";
    RobotArmStateData_.umapAxisStatusString_[7] = "";
    RobotArmStateData_.umapAxisStatusString_[8] = "";
    RobotArmStateData_.umapAxisStatusString_[9] = "";
    RobotArmStateData_.umapAxisStatusString_[10] = "";
    RobotArmStateData_.umapAxisStatusString_[11] = "";
    RobotArmStateData_.umapAxisStatusString_[12] = "";
    RobotArmStateData_.umapAxisStatusString_[13] = ""; // 推杆轴
    RobotArmStateData_.umapAxisStatusString_[14] = ""; // 大臂传送带轴
    RobotArmStateData_.umapAxisStatusString_[15] = ""; // 大臂传送带轴

    RobotArmStateData_.umapAxisMtype_[0] = 0;
    RobotArmStateData_.umapAxisMtype_[1] = 0;
    RobotArmStateData_.umapAxisMtype_[2] = 0;
    RobotArmStateData_.umapAxisMtype_[3] = 0;
    RobotArmStateData_.umapAxisMtype_[4] = 0;
    RobotArmStateData_.umapAxisMtype_[5] = 0;
    RobotArmStateData_.umapAxisMtype_[6] = 0;
    RobotArmStateData_.umapAxisMtype_[7] = 0;
    RobotArmStateData_.umapAxisMtype_[8] = 0;
    RobotArmStateData_.umapAxisMtype_[9] = 0;
    RobotArmStateData_.umapAxisMtype_[10] = 0;
    RobotArmStateData_.umapAxisMtype_[11] = 0;
    RobotArmStateData_.umapAxisMtype_[12] = 0;
    RobotArmStateData_.umapAxisMtype_[13] = 0;
    RobotArmStateData_.umapAxisMtype_[14] = 0;
    RobotArmStateData_.umapAxisMtype_[15] = 0;
}


RobotArm::~RobotArm()
{
    ZMC_Handle_ = nullptr;
    StopUpdateThread();

    WorkBase::Stop();
    if (ZMC_Handle_ != nullptr)
    {
        ZAux_Close(ZMC_Handle_);
        ZMC_Handle_ = nullptr;
        RobotArmStateData_.RobotArmState_ = RobotArmState::Uninitialized;
    }
}

void RobotArm::WorkFun()
{
    // 任务循环
    if(ZMC_Handle_ != nullptr && RobotArmStateData_.RobotArmState_ == RobotArmState::Normal)
    {
        // ------------状态功能执行------------
        if (bRobotArmPause_ && RobotArmStateData_.ExecuteState_ != ExecuteState::Pause)
        {
            // 重新 备份一遍
            RobotArmStateData_.ExecuteState_ = ExecuteState::Pause;
        }

        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Pause)
        {
            return;
        }

        // 已经完成任务
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Unload_Arrive)  // 102:完成
        {
            if (bRobotArmResume_)
            {
                bRobotArmResume_ = false;
            }
            RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;
            pStateCallback_(RobotArmStateData_);
            return;
        }

        // 抓取-正在移动 状态处理
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Unload_MoveingtoTarget)
        {
            if(isArriveThisPlace(TargetAxisReal_))
            {
                // 判断距离，根据点测距的数据 改变深度值 X
                double dToBoxDis = GetPointLaserLongDistance();
                dToBoxDis = 109;
                if (((dToBoxDis < 108) || (dToBoxDis > 110)) && !bDepthOffset_) // +- 2mm
                {
                    bDepthOffset_ = true;
                    std::stringstream log_data;
                    log_data << u8"补偿前抓取位置点 x:" << TargetPose_.x_ << u8" y:" << TargetPose_.y_ \
                             << u8" z:" << TargetPose_.z_
                             << u8" pitch_:" << TargetPose_.pitch_ / (M_PI / 180)
                             << u8" roll:" << TargetPose_.roll_ / (M_PI / 180)
                             << u8" yaw:" << TargetPose_.yaw_ / (M_PI / 180)
                             << u8" dToBoxDis " << dToBoxDis;
                    log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);

                   TargetPose_.x_ +=  dToBoxDis - 109; // 118  250是这款机械臂推荐的抓取距离
                   float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                           , (float)(TargetPose_.roll_ / (M_PI / 180))
                                           , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                           , (float)(TargetPose_.yaw_ / (M_PI / 180))};

                   MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);

                   std::stringstream log_data1;
                   log_data1<< u8"补偿后抓取位置点 x:" << TargetPose_.x_ << u8" y:" << TargetPose_.y_
                            << u8" z:" << TargetPose_.z_
                            << u8" pitch_:" << TargetPose_.pitch_ / (M_PI / 180)
                            << u8" roll:" << TargetPose_.roll_ / (M_PI / 180)
                            << u8" yaw:" << TargetPose_.yaw_ / (M_PI / 180)
                            << u8" dToBoxDis " << dToBoxDis;
                   log_.AddLog(__LINE__, __FILE__, log_data1.str().c_str(), log_data1.str().size(), LLOG::TLogLevel::DEBUG);
                   return;
                }

                std::stringstream log_data;
                log_data << u8"抓取位置点 x:" << TargetPose_.x_ << u8" y:" << TargetPose_.y_
                         << u8" z:" << TargetPose_.z_
                         << u8" pitch_:" << TargetPose_.pitch_ / (M_PI / 180)
                         << u8" roll:" << TargetPose_.roll_ / (M_PI / 180)
                         << u8" yaw:" << TargetPose_.yaw_ / (M_PI / 180);
                log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);


                if ((ExecutedBoxLabel_ == 1)
                    || (ExecutedBoxLabel_ == 6)
                    || (ExecutedBoxLabel_ == 3))
                {   // 左侧
                    TcpPush();  // 吐出吸盘 不回零
                    iTuboMode_ = 3; // 吸气
                    iMode_12_ = 0;  // 小臂皮带不用动
                    iMode_14_ = 1;  // 大臂皮带一直往后走
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_AvrisCorrect_Suck_up;
                    pStateCallback_(RobotArmStateData_);
                }
                else if((ExecutedBoxLabel_ == 2)
                    || (ExecutedBoxLabel_ == 7)
                    || (ExecutedBoxLabel_ == 4))
                {   // 右侧
                    TcpPush();  // 吐出吸盘 不回零
                    iTuboMode_ = 3; // 吸气
                    iMode_12_ = 0;  // 小臂皮带不用动
                    iMode_14_ = 1;  // 大臂皮带一直往后走
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_AvrisCorrect_Suck_up;
                    pStateCallback_(RobotArmStateData_);
                }
                else
                {
                    // 到达位置 并 开始尝试抓取箱体
                    iTuboMode_ = 1;
                    iMode_12_ = 1;
                    iMode_14_ = 1;
                    bTryGrasp_ = false;
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_SingleGrasp;
                    pStateCallback_(RobotArmStateData_);
                    llGraspWaitTime_ = 0;
                    tcp_single_run();
                }
            }
            return;
        }

        // 侧边箱体纠正 吸住箱体 上提
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Unload_AvrisCorrect_Suck_up)
        {
            iTuboMode_ = 3; // 吸气
            iMode_12_ = 0;  // 小臂皮带不用动
            iMode_14_ = 1;  // 大臂皮带一直往后走

            // 判断是否 完成吸住
            if (fDpos_13_ >= 620)    // 基本认为到位了
            {
                // 移动机械臂
                float out[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                        , (float)(TargetPose_.roll_ / (M_PI / 180))
                                        , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                        , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                if ((ExecutedBoxLabel_ == 1)
                    || (ExecutedBoxLabel_ == 6)
                    || (ExecutedBoxLabel_ == 3))
                {   // 左侧
                    out[2] += 30;
                }
                else if((ExecutedBoxLabel_ == 2)
                    || (ExecutedBoxLabel_ == 7)
                    || (ExecutedBoxLabel_ == 4))
                {   // 右侧
                    out[2] += 30;
                }

                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                MoveAbs(ZMC_Handle_, 6, axis_rt_list, out, TargetAxisReal_);
                RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_AvrisCorrect_Suck;
                pStateCallback_(RobotArmStateData_);
                llGraspWaitTime_ = 0;
            }
            return;
        }

        // 侧边箱体纠正 吸住箱体
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Unload_AvrisCorrect_Suck)
        {
            iTuboMode_ = 3; // 吸气
            iMode_12_ = 0;  // 小臂皮带不用动
            iMode_14_ = 1;  // 大臂皮带一直往后走

            // 判断是否 完成上提
            if (isArriveThisPlace(TargetAxisReal_))
            {
                // 移动机械臂
                float out[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                        , (float)(TargetPose_.roll_ / (M_PI / 180))
                                        , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                        , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                if ((ExecutedBoxLabel_ == 1)
                    || (ExecutedBoxLabel_ == 6)
                    || (ExecutedBoxLabel_ == 3))
                {   // 左侧
                    out[1] += -150;
                    out[5] = 8;
                    out[2] += 40;
                }
                else if((ExecutedBoxLabel_ == 2)
                    || (ExecutedBoxLabel_ == 7)
                    || (ExecutedBoxLabel_ == 4))
                {   // 右侧
                    out[1] += 150;
                    out[5] = -8;
                    out[2] += 40;
                }
                TargetPose_.SetPose(out[0], out[1], out[2] - 40
                        , out[4] * (M_PI / 180)
                        , out[3] * (M_PI / 180)
                        , out[5] * (M_PI / 180));

                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                MoveAbs(ZMC_Handle_, 6, axis_rt_list, out, TargetAxisReal_);
                RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_AvrisCorrect_Drag;
                pStateCallback_(RobotArmStateData_);
                llGraspWaitTime_ = 0;
            }
            return;
        }

        // 侧边箱体纠正 拖动箱体
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Unload_AvrisCorrect_Drag)
        {
            iTuboMode_ = 3; // 吸气
            iMode_12_ = 0;  // 小臂皮带不用动
            iMode_14_ = 1;  // 大臂皮带一直往后走
            // 判断是否到位
            if (isArriveThisPlace(TargetAxisReal_))
            {
                // 收回吸盘
                iTuboMode_ = 4;
                if (llGraspWaitTime_ == 0)
                {
                    llGraspWaitTime_ = std::chrono::duration_cast<std::chrono::milliseconds> \
                                       (std::chrono::system_clock::now().time_since_epoch()).count();
                    return;
                }
                else if ((std::chrono::duration_cast<std::chrono::milliseconds> \
                            (std::chrono::system_clock::now().time_since_epoch()).count() - llGraspWaitTime_) < 150)
                {
                    return;
                }
                llGraspWaitTime_ = 0;
                // 收回吸盘
                BigarmSuctionCupBracket_.SingleMoveSp(ZMC_Handle_, -85); // 吸盘架全行程 dpos 是 640
                BigarmSuctionCupBracket_.MoveDelay(ZMC_Handle_, 300);
                // 移动机械臂 后退 20  往侧边 130
                if ((ExecutedBoxLabel_ == 1)
                    || (ExecutedBoxLabel_ == 6)
                    || (ExecutedBoxLabel_ == 3))
                {   // 左侧
                    TargetPose_.x_ -= RobotArmStateData_.fLeftEdgeReturnXDis_;
                    TargetPose_.y_ += RobotArmStateData_.fLeftEdgeReturnDis_;
                    TargetPose_.yaw_ = RobotArmStateData_.fLeftEdgeReturnAngle_ * (M_PI / 180);
                }
                else if((ExecutedBoxLabel_ == 2)
                    || (ExecutedBoxLabel_ == 7)
                    || (ExecutedBoxLabel_ == 4))
                {   // 右侧
                    TargetPose_.x_ -= RobotArmStateData_.fRightEdgeReturnXDis_;
                    TargetPose_.y_ -= RobotArmStateData_.fRightEdgeReturnDis_;
                    TargetPose_.yaw_ = RobotArmStateData_.fRightEdgeReturnAngle_ * (M_PI / 180);
                }

                float in[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                        , (float)(TargetPose_.roll_ / (M_PI / 180))
                                        , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                        , (float)(TargetPose_.yaw_ / (M_PI / 180))};

                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                MoveAbs(ZMC_Handle_, 6, axis_rt_list, in, TargetAxisReal_);
                RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_AvrisCorrect_GoToGraspPoint;
                pStateCallback_(RobotArmStateData_);
            }

            return;
        }

        // 侧边箱体纠正 去到抓取点
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Unload_AvrisCorrect_GoToGraspPoint)
        {
            iTuboMode_ = 4; // 吹气
            iMode_12_ = 0;  // 小臂皮带不用动
            iMode_14_ = 1;  // 大臂皮带一直往后走

            bool bPushrodHoming = false;

            if (fDpos_13_ <= 545)
            {
                bPushrodHoming = true;
            }
            // 判断是否到位
            if (isArriveThisPlace(TargetAxisReal_) && bPushrodHoming)
            {
                iTuboMode_ = 5; // 吸气
                iMode_12_ = 5;
                VacuumChuckFunction();
                iMode_14_ = 1;  // 大臂皮带一直往后走
                bTryGrasp_ = false;
                RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_SingleGrasp;
                pStateCallback_(RobotArmStateData_);
                llGraspWaitTime_ = 0;
                tcp_single_run(fDpos_13_);
            }

            return;
        }

        // 恢复运行
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Resume)
        {
            bool bTcpResume = false;
            // 判断吸盘是否回归
            if (!GetPhotoSensorIsTriggered(9) && bRobotArmResume_)
            {
                // 吸盘回归
                TcpHoming();
            }
            else if (GetPhotoSensorIsTriggered(9))
            {
                bTcpResume = true;
            }

            // 关闭全部 IO
            for (int i = 0; i < (int)RobotArmStateData_.umapIoOut_.size(); ++i)
            {
                if (RobotArmStateData_.umapIoOut_[i])
                {
                    IoOut(i, false);
                }
            }

            if (bTcpResume)
            {   // 完成 恢复流程
                if (PauseBackupData_.ExecuteState_ > ExecuteState::Unload_StateBegin
                    && PauseBackupData_.ExecuteState_ < ExecuteState::Unload_StateEndl)
                {   // 停止前 是 卸货
                    if (PauseBackupData_.ExecuteState_ == ExecuteState::Unload_MoveingtoTarget)
                    { // 正在前往目标点
                        bDepthOffset_ = false;
                        TargetPose_ = PauseBackupData_.TargetPose_; // 暂停前的目标位置
                        float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                                , (float)(TargetPose_.roll_ / (M_PI / 180))
                                                , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                                , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                        MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
                        RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_MoveingtoTarget;
                    }
                    else if (PauseBackupData_.ExecuteState_ == ExecuteState::Unload_GraspingDone)
                    {
                        RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_GraspingDone;
                    }
                    else if (PauseBackupData_.ExecuteState_ == ExecuteState::Unload_SingleGrasp
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Unload_AvrisCorrect_Suck
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Unload_AvrisCorrect_Suck_up
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Unload_AvrisCorrect_Drag
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Unload_AvrisCorrect_GoToGraspPoint)
                    {
                        bDepthOffset_ = false;
                        ExecutedBox_ = PauseBackupData_.ExecutedBox_;
                        TargetPose_ = PauseBackupData_.ExecutedBox_.LoadPose_; // 暂停前的目标位置
                        float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                                , (float)(TargetPose_.roll_ / (M_PI / 180))
                                                , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                                , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                        MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
                        RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_MoveingtoTarget;
                    }
                    else
                    {
                        RobotArmStateData_.ExecuteState_ = PauseBackupData_.ExecuteState_;
                    }
                }
                else if (PauseBackupData_.ExecuteState_ > ExecuteState::Upload_StateBegin
                    && PauseBackupData_.ExecuteState_ < ExecuteState::Upload_StateEndl)
                {   // 停止前 是 上货
                    if (PauseBackupData_.ExecuteState_ == ExecuteState::Upload_MoveingtoTarget)
                    { // 正在前往目标点
                        TargetPose_ = PauseBackupData_.TargetPose_; // 暂停前的目标位置
                        float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                                , (float)(TargetPose_.roll_ / (M_PI / 180))
                                                , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                                , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                        MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
                        RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_MoveingtoTarget;
                    }
                    else if (PauseBackupData_.ExecuteState_ == ExecuteState::Upload_SingleLoadDone)
                    {
                        RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoadDone;
                    }
                    else if (PauseBackupData_.ExecuteState_ == ExecuteState::Upload_SingleLoading
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_RightSideCorrecting
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_EjectionBox
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_Correct_ToReadyPosition
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_Correct_SwingLever
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_Correct_BackOffCorrecting
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_Correct_ReturnPosition_MoveLeft
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_Correct_ReturnPosition
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_LengthwaysPush)
                    {
                        ExecutedBox_ = PauseBackupData_.ExecutedBox_;
                        TargetPose_ = PauseBackupData_.ExecutedBox_.LoadPose_; // 暂停前的目标位置
                        float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                                , (float)(TargetPose_.roll_ / (M_PI / 180))
                                                , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                                , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                        MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
                        RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_MoveingtoTarget;
                    }
                    else if (PauseBackupData_.ExecuteState_ == ExecuteState::Upload_LoadingDone_MoveBackward
                             || PauseBackupData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_MoveBackward)
                    {
                        TargetPose_ = PauseBackupData_.TargetPose_; // 暂停前的目标位置
                        float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                                , (float)(TargetPose_.roll_ / (M_PI / 180))
                                                , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                                , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                        MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
                        RobotArmStateData_.ExecuteState_ = PauseBackupData_.ExecuteState_;
                    }
                    else
                    {
                        RobotArmStateData_.ExecuteState_ = PauseBackupData_.ExecuteState_;
                    }
                }
                else if (PauseBackupData_.ExecuteState_ == ExecuteState::SinglePointMove)
                {   // 停止前 是 单独运动中
                    TargetPose_ = PauseBackupData_.TargetPose_; // 暂停前的目标位置
                    float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                            , (float)(TargetPose_.roll_ / (M_PI / 180))
                                            , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                            , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                    MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
                    RobotArmStateData_.ExecuteState_ = PauseBackupData_.ExecuteState_;
                }
                else    // 停止前 是 空闲、完成等
                {
                    RobotArmStateData_.ExecuteState_ = PauseBackupData_.ExecuteState_;
                }
            }

            bRobotArmResume_ = false;
            return;
        }

        // 单个抓取
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Unload_SingleGrasp)
        {
            if (bRobotArmResume_)
            {
                bRobotArmResume_ = false;
            }

            if (fDpos_13_ > 15.0)
            {
                bTryGrasp_ = true;
                llGraspWaitTime_ = 0;
            }

            bool bGraspingDone = false; // 小臂光电是否触发

            if (GetPhotoSensorIsTriggered(20)
                || GetPhotoSensorIsTriggered(21)
                || GetPhotoSensorIsTriggered(22)
                || GetPhotoSensorIsTriggered(19)
                || GetPhotoSensorIsTriggered(18))
            {
                bGraspingDone = true;
            }

            bool bPushrodHoming = false; // 推杆归位
            bPushrodHoming = RobotArmStateData_.umapIoInput_[9] > 0 ? true : false;

            // llSendTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            if (bPushrodHoming && bTryGrasp_)
            {
                iTuboMode_ = 1;
                iMode_12_ = 1;
                if (!bGraspingDone)
                {   // 14 15 端口没有监测到箱体  等待500 毫秒 还没有，说明抓取失败
                    if (GetPhotoSensorIsTriggered(10)
                        || GetPhotoSensorIsTriggered(11)
                        || GetPhotoSensorIsTriggered(12)
                        || GetPhotoSensorIsTriggered(13)
                        || GetPhotoSensorIsTriggered(14)
                        || GetPhotoSensorIsTriggered(15)
                        || GetPhotoSensorIsTriggered(16)
                        || GetPhotoSensorIsTriggered(17))
                    {
                        return;
                    }

                    if (llGraspWaitTime_ != 0)
                    {
                        if (std::chrono::duration_cast<std::chrono::milliseconds> \
                                (std::chrono::system_clock::now().time_since_epoch()).count()
                            - llGraspWaitTime_ > 1500)
                        {   // 抓取失败
                            // 从新抓取  失败就再次抓取
                            llGraspWaitTime_ = 0;
                            iTuboMode_ = 1;
                            tcp_single_run();
                        }
                    }
                    else
                    {   // 获取时间，开始时间
                        llGraspWaitTime_ = std::chrono::duration_cast<std::chrono::milliseconds> \
                                (std::chrono::system_clock::now().time_since_epoch()).count();
                    }
                }
                else if (GetPhotoSensorIsTriggered(10)
                         || GetPhotoSensorIsTriggered(11)
                         || GetPhotoSensorIsTriggered(12)
                         || GetPhotoSensorIsTriggered(13)
                         || GetPhotoSensorIsTriggered(14)
                         || GetPhotoSensorIsTriggered(15)
                         || GetPhotoSensorIsTriggered(16)
                         || GetPhotoSensorIsTriggered(17))
                {
                    return;
                }
                else
                {   // 小臂与大臂交界处的光电监测到箱体
                    // 完成抓取
                    llGraspWaitTime_ = 0;
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_GraspingDone;
                    return;
                }
            }
            return;
        }

        // 抓取完成
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Unload_GraspingDone)
        {
            if (!qUnloadBox_.empty())
            {
                ExecutedBox_ = qUnloadBox_.front();
                qUnloadBox_.pop();
                ExecutedBoxLabel_ = ExecutedBox_.iLabel_;
                TargetPose_ = ExecutedBox_.OperationPose_;
                float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                        , (float)(TargetPose_.roll_ / (M_PI / 180))
                                        , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                        , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                bDepthOffset_ = false;
                MoveAbs(ZMC_Handle_,6,axis_rt_list,dis, TargetAxisReal_);
                RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_MoveingtoTarget;
                std::stringstream log_data;
                log_data << u8"抓取位置点 x:" << TargetPose_.x_ << u8" y:" << TargetPose_.y_ << u8" z:" << TargetPose_.z_;
                log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);
            }
            else
            {   // 已完成队列中所有的任务
                RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_Arrive;
                pStateCallback_(RobotArmStateData_);
                RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;
            }
            return;
        }

        //装货单个完成
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoadDone)
        {
            if (!qUnloadBox_.empty())
            {
                ExecutedBox_ = qUnloadBox_.front();
                qUnloadBox_.pop();
                ExecutedBoxLabel_ = ExecutedBox_.iLabel_;
                TargetPose_ = ExecutedBox_.OperationPose_;
                float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                        , (float)(TargetPose_.roll_ / (M_PI / 180))
                                        , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                        , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                bDepthOffset_ = false;
                MoveAbs(ZMC_Handle_,6,axis_rt_list,dis, TargetAxisReal_);
                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_MoveingtoTarget;
                std::stringstream log_data;
                log_data << u8"抓取位置点 x:" << TargetPose_.x_ << u8" y:" << TargetPose_.y_ \
                         << u8" z:" << TargetPose_.z_;
                log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);
            }
            else
            {   // 已完成队列中所有的任务
                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_Arrive;
                pStateCallback_(RobotArmStateData_);
                RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;
            }
            return;
        }

        // 装货-正在移动
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_MoveingtoTarget)
        {
            // 大臂皮带
            if(!GetPhotoSensorIsTriggered(22)) // 大臂与小臂之间的光电传感器，后面需要改
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }   // 大小臂间有货物
            else if (!GetPhotoSensorIsTriggered(10)
                     && !GetPhotoSensorIsTriggered(11)
                     && !GetPhotoSensorIsTriggered(12)
                     && !GetPhotoSensorIsTriggered(13)
                     && !GetPhotoSensorIsTriggered(14)
                     && !GetPhotoSensorIsTriggered(15)
                     && !GetPhotoSensorIsTriggered(16)
                     && !GetPhotoSensorIsTriggered(17)
                     && !GetPhotoSensorIsTriggered(18)
                     && !GetPhotoSensorIsTriggered(19)  // 小臂皮带没有东西
                     && GetPhotoSensorIsTriggered(9) )   // 推杆处于等待位
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }
            else    // 大臂皮带末端有货，并且小臂：有货或推杆推出
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }

            // 小臂皮带也要慢转
            if (GetPhotoSensorIsTriggered(10)
                || GetPhotoSensorIsTriggered(11)
                || GetPhotoSensorIsTriggered(12)
                || GetPhotoSensorIsTriggered(13)
                || GetPhotoSensorIsTriggered(14)
                || GetPhotoSensorIsTriggered(15)
                || GetPhotoSensorIsTriggered(16)
                || GetPhotoSensorIsTriggered(17)
                || GetPhotoSensorIsTriggered(18)
                || GetPhotoSensorIsTriggered(19)
                || !GetPhotoSensorIsTriggered(9) ) // 推杆 推出
            {
                iMode_12_ = 0;
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_12_ = 2;
                BigarmConveyorBelt_.SetSpeed(ZMC_Handle_, 400.0);
                BigarmConveyorBelt_.ContinueRun(ZMC_Handle_, -1);
            }
            if(isArriveThisPlace(TargetAxisReal_))
            {
                // --------------- 不需要补偿 ----------------
                // 判断距离，根据点测距的数据 改变深度值 X
//                double dToBoxDis = GetPointLaserShortDistance();    // 获取短距离测距
                //如果箱体长度  box_length = 200mm
//                dToBoxDis = 200; // todo test     测试时需要更改
//                if ((dToBoxDis < box_length -2 || dToBoxDis > box_length+2) && !bDepthOffset_) // +- 2mm
//                {
//                    bDepthOffset_ = true;
//                    std::stringstream log_data;
//                    log_data << u8"补偿前装货位置点 x:" << TargetPose_.x_ << u8" y:" << TargetPose_.y_
//                             << u8" z:" << TargetPose_.z_
//                             << u8" pitch_:" << TargetPose_.pitch_
//                             << u8" roll:" << TargetPose_.roll_
//                             << u8" yaw:" << TargetPose_.yaw_
//                             << u8" dToBoxDis " << dToBoxDis;
//                    log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);

//                   TargetPose_.x_ +=  dToBoxDis - 111 - box_length; // 111是这款机械臂推荐的抓取距离
//                   float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_,
//                                     (float)TargetPose_.roll_, (float)TargetPose_.pitch_, (float)TargetPose_.yaw_};

//                   ZAux_Direct_MoveAbs(ZMC_Handle_,6,axis_rt_list,dis);
//                   MoveAbs(ZMC_Handle_,6,axis_rt_list,dis);

//                   std::stringstream log_data1;
//                   log_data1<< u8"补偿后装货位置点 x:" << TargetPose_.x_ << u8" y:" << TargetPose_.y_
//                            << u8" z:" << TargetPose_.z_
//                            << u8" pitch_:" << TargetPose_.pitch_
//                            << u8" roll:" << TargetPose_.roll_
//                            << u8" yaw:" << TargetPose_.yaw_
//                            << u8" dToBoxDis " << dToBoxDis;
//                   log_.AddLog(__LINE__, __FILE__, log_data1.str().c_str(), log_data1.str().size(), LLOG::TLogLevel::DEBUG);
//                   return;
//                }
                // --------------- 不需要补偿 end----------------

                std::stringstream log_data;
                log_data << u8"到达装货位置点 x:" << TargetPose_.x_
                         << " y:" << TargetPose_.y_
                         << " z:" << TargetPose_.z_
                         << " pitch_:" << TargetPose_.pitch_ / (M_PI / 180)
                         << " roll:" << TargetPose_.roll_ / (M_PI / 180)
                         << " yaw:" << TargetPose_.yaw_ / (M_PI / 180);
                log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);


                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading;

                // TargetPose_.SetZero();
                // 到达位置 并 开始尝试装箱
//                iTuboMode_ = 0;
//                iMode_12_ = 2;
//                iMode_14_ = 2; //大臂传送带装货变速运动
//                bTryGrasp_ = false;
//                bLoadingDone_ = false;
//                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading;
//                pStateCallback_(RobotArmStateData_);
//                llGraspWaitTime_ = 0;
            }
            return;
        }

        // 单个装货
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading)
        {
            // 识别小臂上是不是有 货物
            // 如果有货物 就加速把货物甩出去
            // 如果没有货物就让大/小臂动起来
            if(!GetPhotoSensorIsTriggered(22)) // 大臂与小臂之间的光电传感器
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }   // 大小臂间有货物
            else if (!GetPhotoSensorIsTriggered(10)
                     && !GetPhotoSensorIsTriggered(11)
                     && !GetPhotoSensorIsTriggered(12)
                     && !GetPhotoSensorIsTriggered(13)
                     && !GetPhotoSensorIsTriggered(14)
                     && !GetPhotoSensorIsTriggered(15)
                     && !GetPhotoSensorIsTriggered(16)
                     && !GetPhotoSensorIsTriggered(17)
                     && !GetPhotoSensorIsTriggered(18)
                     && !GetPhotoSensorIsTriggered(19)  // 小臂皮带没有东西
                     && GetPhotoSensorIsTriggered(9) )   // 推杆处于等待位
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }
            else    // 大臂皮带末端有货，并且小臂：有货或推杆推出
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);

                BackConveyorBelt_.Stop(ZMC_Handle_);
            }

            // 小臂运动
            if (!GetPhotoSensorIsTriggered(10)
                && !GetPhotoSensorIsTriggered(11)
                && !GetPhotoSensorIsTriggered(12)
                && !GetPhotoSensorIsTriggered(13)
                && !GetPhotoSensorIsTriggered(14)
                && !GetPhotoSensorIsTriggered(15)
                && !GetPhotoSensorIsTriggered(16)
                && !GetPhotoSensorIsTriggered(17)
                && !GetPhotoSensorIsTriggered(18)
                && !GetPhotoSensorIsTriggered(19))
            {
                iMode_12_ = 2;
                BigarmConveyorBelt_.SetSpeed(ZMC_Handle_, 400);
                BigarmConveyorBelt_.ContinueRun(ZMC_Handle_, -1);
            }
            else if ((GetPhotoSensorIsTriggered(10)
                  || GetPhotoSensorIsTriggered(11)
                  || GetPhotoSensorIsTriggered(12)
                  || GetPhotoSensorIsTriggered(13)
                  || GetPhotoSensorIsTriggered(14)
                  || GetPhotoSensorIsTriggered(15)
                  || GetPhotoSensorIsTriggered(16)
                  || GetPhotoSensorIsTriggered(17)
                  || GetPhotoSensorIsTriggered(18)
                  || GetPhotoSensorIsTriggered(19))
                  && GetPhotoSensorIsTriggered(9) ) // 需要防止 推杆导致误以为有货
            {
                // 小臂皮带加速  将货物抛出
                iMode_12_ = 2;
                BigarmConveyorBelt_.SetSpeed(ZMC_Handle_, 1200);
                BigarmConveyorBelt_.ContinueRun(ZMC_Handle_, -1);
                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_EjectionBox;
            }
            return;
        }

        //单个正在装货-将货物抛出去
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_EjectionBox)
        {
            // 判断是不是已经抛出
            if ( !GetPhotoSensorIsTriggered(10)
                 && !GetPhotoSensorIsTriggered(11)
                 && !GetPhotoSensorIsTriggered(12)
                 && !GetPhotoSensorIsTriggered(13)
                 && !GetPhotoSensorIsTriggered(14)
                 && !GetPhotoSensorIsTriggered(15)
                 && !GetPhotoSensorIsTriggered(16)
                 && !GetPhotoSensorIsTriggered(17)
                 && !GetPhotoSensorIsTriggered(18)
                 && !GetPhotoSensorIsTriggered(19)
                 && !GetPhotoSensorIsTriggered(20)
                 && !GetPhotoSensorIsTriggered(21))
            {
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }

            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
                iMode_14_ = 2;
            }

            if ( !GetPhotoSensorIsTriggered(10)
                 && !GetPhotoSensorIsTriggered(11)
                 && !GetPhotoSensorIsTriggered(12)
                 && !GetPhotoSensorIsTriggered(13)
                 && !GetPhotoSensorIsTriggered(14)
                 && !GetPhotoSensorIsTriggered(15)
                 && !GetPhotoSensorIsTriggered(16)
                 && !GetPhotoSensorIsTriggered(17)
                 && !GetPhotoSensorIsTriggered(18)
                 && !GetPhotoSensorIsTriggered(19)
                 && !GetPhotoSensorIsTriggered(20)
                 && !GetPhotoSensorIsTriggered(21))
            {
                // 判断 当前安装箱子是不是需要拨正
                if (ExecutedBox_.bNeedCorrected_)
                {
                    if (!bRepeatFunction_)
                    {   // 不是重复进入
                        TargetPose_ = ExecutedBox_.LoadPose_;
                        float in[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                                , (float)(TargetPose_.roll_ / (M_PI / 180))
                                                , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                                , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                        float out[6] = {};
                        ComputeTcpPose(in, 1, ExecutedBox_.Width + 30, 4, 0, out);
                        TargetPose_.SetPose(out[0], out[1], out[2]
                                , out[4] * (M_PI / 180)
                                , out[3] * (M_PI / 180)
                                , out[5] * (M_PI / 180));
                        bRepeatFunction_ = true;
                    }

                    float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                            , (float)(TargetPose_.roll_ / (M_PI / 180))
                                            , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                            , (float)(TargetPose_.yaw_ / (M_PI / 180))};

                    RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                    qDebug() << "机械臂原本需要去到的位置" << dis[6] ;
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_ManualMove;
                    if (MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_))
                    {   // 执行成功
                        bRepeatFunction_ = false;
                        RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_MovingLeft;
                    }
                }
                else
                {   // 不需要 向右动一下
                    if (!bRepeatFunction_)
                    {
                        TargetPose_.y_ -= 135; // -125  480 - ExecutedBox_.Width;
//                        TargetPose_.z_ -= 10;
                        bRepeatFunction_ = true;
                    }
                    float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                            , (float)(TargetPose_.roll_ / (M_PI / 180))
                                            , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                            , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                    RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解

                    if (MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_))
                    {   // 执行成功
                        bRepeatFunction_ = false;
                        RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_RightSideCorrecting;
                    }
                }
            }

            return;
        }
        //单个正在装货-右侧纠正-小臂左移
        if(RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_MovingLeft)
        {
            //  小臂皮带停止转动
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }
            // 大臂皮带有货的话就停
            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }
            if(isArriveThisPlace(TargetAxisReal_))
            {   // 到达位置 右侧挡板伸出
                IoOut(23, false);
                llWaitingTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_BaffleLaunching;
            }
            return;
        }

        //单个正在装货-右侧纠正-右侧挡板伸出
        if(RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_BaffleLaunching)
        {
            //  小臂皮带停止转动
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }
            // 大臂皮带有货的话就停
            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }
            if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
                    - llWaitingTime_ > 3000
                )
            {   // 3秒后 机械臂往右移动
                GetTcpData();
                float dis[6] = {(float)(RobotArmStateData_.fTcp_x_)
                                ,(float)(ExecutedBox_.LoadPose_.y_ - (555 - ExecutedBox_.Width + 50.0))
                                , (float)RobotArmStateData_.fTcp_z_
                                , (float)RobotArmStateData_.fTcp_rx_
                                , (float)RobotArmStateData_.fTcp_ry_
                                , (float)RobotArmStateData_.fTcp_rz_};

                TargetPose_.SetPose((float)dis[0]
                                    , (float)dis[1]
                                    , (float)dis[2]
                                    , (float)dis[4] * (M_PI / 180)
                                    , (float)dis[3] * (M_PI / 180)
                                    , (float)dis[5] * (M_PI / 180));
                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                qDebug() << "往右移";
                if(MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_))
                {
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_MovingRight;
                }
            return;
        }

        //单个正在装货-右侧纠正-小臂向右移
        if(RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_MovingRight)
        {
            //  小臂皮带停止转动
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }
            // 大臂皮带有货的话就停
            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }
            if(isArriveThisPlace(TargetAxisReal_))
                // 机械臂到位后，收起右侧挡板
            {
                IoOut(23, true);
                llWaitingTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_BaffleStowed;
            }
            return;
        }

        // 单个正在装货－右侧箱子纠正－收起右侧挡板
        if(RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_BaffleStowed)
        {
            //  小臂皮带停止转动
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }
            // 大臂皮带有货的话就停
            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }
            if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
                    - llWaitingTime_ > 3000)
                // 3秒后，机械臂往后300
            {
                GetTcpData();
                float dis[6] = {(float)(RobotArmStateData_.fTcp_x_ - 300)
                                , (float)RobotArmStateData_.fTcp_y_
                                , (float)RobotArmStateData_.fTcp_z_
                                , (float)RobotArmStateData_.fTcp_rx_
                                , (float)RobotArmStateData_.fTcp_ry_
                                , (float)RobotArmStateData_.fTcp_rz_};

                TargetPose_.SetPose((float)dis[0]
                                    , (float)dis[1]
                                    , (float)dis[2]
                                    , (float)dis[4] * (M_PI / 180)
                                    , (float)dis[3] * (M_PI / 180)
                                    , (float)dis[5] * (M_PI / 180));
                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                qDebug() << "往后退";
                MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
                bRepeatFunction_ = false;
                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_BackWarding;
            }
        }

        // 单个正在装货－右侧箱子纠正－小臂后退
        if(RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_BackWarding)
        {
            //  小臂皮带停止转动
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }
            // 大臂皮带有货的话就停
            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }
            if(isArriveThisPlace(TargetAxisReal_))
            {
                // 到达位置后，重新前往装货点
                TargetPose_ = ExecutedBox_.LoadPose_;
                float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                , (float)(TargetPose_.roll_ / (M_PI / 180))
                                , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                if (MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_))
                {   // 成功
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_Correct_ReturnPosition_MoveLeft;
                }
        }


        //单个正在装货-右侧移动纠正
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_RightSideCorrecting)
        {
            // 小臂上不能有货物
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }

            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }

            if(isArriveThisPlace(TargetAxisReal_))
            {   // 到达位置 推杆按压
                tcp_single_run();
                bLoadingDone_ = false;
                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_LengthwaysPush;
            }
            return;
        }

        //单个正在装货-移动到预备姿态
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_Correct_ToReadyPosition)
        {
            // 小臂上不能有货物
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }

            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }

            if(isArriveThisPlace(TargetAxisReal_))
            {
                //切换到正解，并对前臂进行运动 右15度运动
                int ret = ZAux_Direct_Connreframe(ZMC_Handle_,6,axis_rt_list,121,10,6,axis_real_list);
                if (!ret)
                {   // 建立正解成功
//                    float dis[6]={(float)RobotArmStateData_.fTcp_x_, (float)RobotArmStateData_.fTcp_y_
//                                  , (float)RobotArmStateData_.fTcp_z_ ,(float)RobotArmStateData_.fTcp_rx_
//                                  , (float)RobotArmStateData_.fTcp_ry_, (float)RobotArmStateData_.fTcp_rz_};
//                    float new_dis[6];
//                    DirectToInverseFrameTans(dis, -13, new_dis);
//                    MoveAbs(ZMC_Handle_, 6, axis_rt_list, new_dis);
//                    ZAux_Direct_Connreframe(ZMC_Handle_,6,axis_rt_list,121,10,6,axis_real_list);
                    float dis[5]={RobotArmStateData_.fL_1_, RobotArmStateData_.fL_2_, RobotArmStateData_.fL_3_
                                  , RobotArmStateData_.fL_4_, -14.5};
                    TargetAxisReal_[0] = dis[0];
                    TargetAxisReal_[1] = dis[1];
                    TargetAxisReal_[2] = dis[2];
                    TargetAxisReal_[3] = dis[3];
                    TargetAxisReal_[4] = dis[4];
                    TargetAxisReal_[5]= 0;
//                    std::copy(std::begin(dis), std::end(dis), std::begin(TargetAxisReal_));
                    qDebug() << "ZAux_Direct_MoveAbs return :" << ZAux_Direct_MoveAbs(ZMC_Handle_,5,axis_real_list,dis);
                    bRepeatFunction_ = false;
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_Correct_SwingLever;
                }
                else
                {
                    qDebug() << "建立正解 失败";
                }
            }
            return;
        }

        //单个正在装货-摆动前方用于拨正的杆子
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_Correct_SwingLever)
        {
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }

            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }

            if(isArriveThisPlace(TargetAxisReal_))
            {
                // 往后退
//                if (!bRepeatFunction_)
//                {
//                    Common::Pose temp_pos(RobotArmStateData_.fTcp_x_, RobotArmStateData_.fTcp_y_, RobotArmStateData_.fTcp_z_
//                                          ,RobotArmStateData_.fTcp_rx_, RobotArmStateData_.fTcp_ry_, RobotArmStateData_.fTcp_rz_);
//                    temp_pos.x_ -= ExecutedBox_.Length - 100;
//                    TargetPose_ = temp_pos;
//                    bRepeatFunction_ = true;
//                }

//                float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_,
//                                  (float)TargetPose_.roll_, (float)TargetPose_.pitch_, (float)TargetPose_.yaw_};
//                                , (float)RobotArmStateData_.fTcp_y_
                //(float)(RobotArmStateData_.fTcp_x_ - (ExecutedBox_.Length - 100.0)) // 往后退
                GetTcpData();
                float dis[6] = {(float)(RobotArmStateData_.fTcp_x_ - 300)
//                                , (float)(ExecutedBox_.LoadPose_.y_ + ExecutedBox_.Width + 100.0)
                                , (float)RobotArmStateData_.fTcp_y_
                                , (float)RobotArmStateData_.fTcp_z_
                                , (float)RobotArmStateData_.fTcp_rx_
                                , (float)RobotArmStateData_.fTcp_ry_
                                , (float)RobotArmStateData_.fTcp_rz_};

                TargetPose_.SetPose((float)dis[0]
                                    , (float)dis[1]
                                    , (float)dis[2]
                                    , (float)dis[4] * (M_PI / 180)
                                    , (float)dis[3] * (M_PI / 180)
                                    , (float)dis[5] * (M_PI / 180));
                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                qDebug() << "往后退";
                MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);

                bRepeatFunction_ = false;
                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_Correct_BackOffCorrecting;
//                if (MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_))
//                {
//                    bRepeatFunction_ = false;
//                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_Correct_BackOffCorrecting;
//                }
            }

            // 判断是否 已经往右拐，并到
            // 15 的 +-0.1
//            if (RobotArmStateData_.fL_5_ < -12.9 && RobotArmStateData_.fL_5_ > -13.1)
//            {
//                // 往后退
//                if (!bRepeatFunction_)
//                {
//                    Common::Pose temp_pos(RobotArmStateData_.fTcp_x_, RobotArmStateData_.fTcp_y_, RobotArmStateData_.fTcp_z_
//                                          ,RobotArmStateData_.fTcp_rx_, RobotArmStateData_.fTcp_ry_, RobotArmStateData_.fTcp_rz_);
//                    temp_pos.x_ -= ExecutedBox_.Length;
//                    TargetPose_ = temp_pos;
//                    bRepeatFunction_ = true;
//                }

//                float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_,
//                                  (float)TargetPose_.roll_, (float)TargetPose_.pitch_, (float)TargetPose_.yaw_};
//                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
//                if (MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis))
//                {
//                    bRepeatFunction_ = false;
//                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_Correct_BackOffCorrecting;
//                }
//            }

            return;
        }

        // 单个正在装货-后退纠正
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_Correct_BackOffCorrecting)
        {
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }

            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }

            if(isArriveThisPlace(TargetAxisReal_))
            {
//                TargetPose_ = ExecutedBox_.LoadPose_;
//                TargetPose_.z_ -= 20;
                TargetPose_.y_ += 100;
                float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                        , (float)(TargetPose_.roll_ / (M_PI / 180))
                                        , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                        , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                if (MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_))
                {   // 成功

                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_Correct_ReturnPosition_MoveLeft;

                }
//                int ret = ZAux_Direct_Connreframe(ZMC_Handle_,6,axis_rt_list,121,10,6,axis_real_list);
//                if (!ret)
//                {   // 建立正解成功
//                    float dis[5]={RobotArmStateData_.fL_1_, RobotArmStateData_.fL_2_, RobotArmStateData_.fL_3_
//                                  , RobotArmStateData_.fL_4_, 0.0};
//                    ZAux_Direct_MoveAbs(ZMC_Handle_,5,axis_real_list,dis);
//                }
//                else
//                {
//                    qDebug() << "建立正解失败";
//                }

            }
//            {
//                TargetPose_ = ExecutedBox_.LoadPose_;
//                float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_,
//                                  (float)TargetPose_.roll_, (float)TargetPose_.pitch_, (float)TargetPose_.yaw_};
//                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
//                if (MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis))
//                {   // 成功
//                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_Correct_ReturnPosition;
//                }
//            }
            return;
        }
        // 单个正在装货-回到原上货位
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_Correct_ReturnPosition_MoveLeft)
        {
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }

            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }


            if(isArriveThisPlace(TargetAxisReal_))
            {
                TargetPose_ = ExecutedBox_.LoadPose_;
//                TargetPose_.z_ -= 10;
                float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                        , (float)(TargetPose_.roll_ / (M_PI / 180))
                                        , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                        , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                if (MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_))
                {   // 成功

                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_Correct_ReturnPosition;

                }
            }
            return;
        }

        // 单个正在装货-回到原上货位
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_Correct_ReturnPosition)
        {
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }

            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }

            if(isArriveThisPlace(TargetAxisReal_))
            {
                tcp_single_run();
                bLoadingDone_ = false;
                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_LengthwaysPush;
            }
            return;
        }

        // 单个正在装货- 纵向推压
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_LengthwaysPush)
        {
            // 小臂上不能有货物
            if (RobotArmStateData_.umapAxisStatus_[12] == 0)
            {   // 12轴在运动
                iMode_12_ = 0;  // 小臂皮带停止转动
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }

            if (GetPhotoSensorIsTriggered(22))
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }


            if (!GetPhotoSensorIsTriggered(9))
            {
                bLoadingDone_ = true;   // 记录推杆已推出
            }

            if (bLoadingDone_ && GetPhotoSensorIsTriggered(9))    // 确认记录推杆已推出 并且又回到归属位
            {
                bLoadingDone_ = false;
                // 完成 推压说明 完成一个装货

                if (qLoadBox_.size() == 0)  // 判断是不是最后一个
                {
                    // 当前是最后一个 机械臂需要往后缩一点
                    TargetPose_.x_ = ExecutedBox_.LoadPose_.x_ - 350;
                    float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                            , (float)(TargetPose_.roll_ / (M_PI / 180))
                                            , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                            , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                    RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                    MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_LoadingDone_MoveBackward;
                }

                // 如果是最左边的 就先退回一点，避免拨板打到箱子 移动点
                if (ExecutedBox_.BoxPoseType_ == Common::BoxPoseType::Left)
                {
                    // 拨板长度 640  突出来 370
                    TargetPose_.x_ = ExecutedBox_.LoadPose_.x_ - 350;
                    TargetPose_.z_ = ExecutedBox_.LoadPose_.z_ + ExecutedBox_.Height + 15;
                    float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                            , (float)(TargetPose_.roll_ / (M_PI / 180))
                                            , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                            , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                    RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                    MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_SingleLoading_MoveBackward;
                }
                else    // 中间箱子
                {
                    ExecutedBox_ = qLoadBox_.front();
                    qLoadBox_.pop();
                    bDepthOffset_ = false;  // 用于记录X轴是否已经补偿过
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_MoveingtoTarget;    // 移动到装货点
                    // 逆解方式 移动到 装货点
                    TargetPose_ = ExecutedBox_.LoadPose_;
                    float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                            , (float)(TargetPose_.roll_ / (M_PI / 180))
                                            , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                            , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                    RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                    MoveAbs(ZMC_Handle_,6,axis_rt_list,dis, TargetAxisReal_);
                }
            }
            return;
        }

        // 完成一面所需要装载的货物
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_LoadingDone_MoveBackward)
        {
            if(isArriveThisPlace(TargetAxisReal_))
            {   // 机械臂到达需要腾出的位置  返回完成任务
                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_Arrive;
                pStateCallback_(RobotArmStateData_);
                RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;
            }
            return;
        }

        // 单个正在装货-往后走，避免拨杆与箱体干涉
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_MoveBackward)
        {
            if(RobotArmStateData_.umapIoInput_[16] < 1) // 大臂与小臂之间的光电传感器
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }   // 大小臂间有货物
            else if (!GetPhotoSensorIsTriggered(10)
                     && !GetPhotoSensorIsTriggered(11)
                     && !GetPhotoSensorIsTriggered(12)
                     && !GetPhotoSensorIsTriggered(13)
                     && !GetPhotoSensorIsTriggered(14)  // 小臂皮带没有东西
                     && !GetPhotoSensorIsTriggered(15)
                     && !GetPhotoSensorIsTriggered(16)
                     && !GetPhotoSensorIsTriggered(17)
                     && !GetPhotoSensorIsTriggered(18)
                     && !GetPhotoSensorIsTriggered(19)
                     && GetPhotoSensorIsTriggered(9))   // 推杆处于等待位
            {
                iMode_14_ = 2;
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }
            else    // 大臂皮带末端有货，并且小臂：有货或推杆推出
            {
                iMode_14_ = 0;
                ForearmConveyorBelt_.Stop(ZMC_Handle_);
                BackConveyorBelt_.Stop(ZMC_Handle_);
            }

            // 小臂皮带也要慢转
            if (GetPhotoSensorIsTriggered(10)
               || GetPhotoSensorIsTriggered(11)
               || GetPhotoSensorIsTriggered(12)
               || GetPhotoSensorIsTriggered(13)
               || GetPhotoSensorIsTriggered(14)
               || GetPhotoSensorIsTriggered(15)
               || GetPhotoSensorIsTriggered(16)
               || GetPhotoSensorIsTriggered(17)
               || GetPhotoSensorIsTriggered(18)
               || GetPhotoSensorIsTriggered(19)
               || !GetPhotoSensorIsTriggered(9)) // 推杆 推出
            {
                iMode_12_ = 0;
                BigarmConveyorBelt_.Stop(ZMC_Handle_);
            }
            else
            {
                iMode_12_ = 2;
                BigarmConveyorBelt_.SetSpeed(ZMC_Handle_, 400.0);
                BigarmConveyorBelt_.ContinueRun(ZMC_Handle_, -1);
            }

            if(isArriveThisPlace(TargetAxisReal_))
            {
                // 移动到 下一个箱体
                ExecutedBox_ = qLoadBox_.front();
                qLoadBox_.pop();
                bDepthOffset_ = false;  // 用于记录X轴是否已经补偿过
                RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_MoveingtoTarget;    // 移动到装货点
                // 逆解方式 移动到 装货点
                TargetPose_ = ExecutedBox_.LoadPose_;
                float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                        , (float)(TargetPose_.roll_ / (M_PI / 180))
                                        , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                        , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
                MoveAbs(ZMC_Handle_,6,axis_rt_list,dis, TargetAxisReal_);
            }
            return;
        }

        // 卸货模式：拍照-正在移动 状态处理
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::SinglePointMove)
        {
            if(isArriveThisPlace(TargetAxisReal_))
            {
                RobotArmStateData_.ExecuteState_ = ExecuteState::Arrive;
                pStateCallback_(RobotArmStateData_);
                RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;
            }
            return;
        }

        // 测试模式 - 推杆按次数运动
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::TestMode_TCPReciprocating_Count)
        {
            // 判断是否需要执行推杆
            if (!bTryGrasp_)
            {   // 没有尝试 抓取（执行推杆）
                if (GetPhotoSensorIsTriggered(9) && (iTestModeRunCount_ > 0))
                {
                    tcp_single_run();
                    // 判断 皮带是否需要动
                    if (bTestModeEnablementConveyor_)
                    {
                        iTuboMode_ = 1;
                        iMode_12_ = 1;
                    }
                    else
                    {
                        iTuboMode_ = 0;
                        iMode_12_ = 0;
                    }
                    iTestModeRunCount_--;
                    bTryGrasp_ = true;
                    bIsPushedOut_ = false;
                }
                else if (iTestModeRunCount_ < 1)
                {
                    iTuboMode_ = 0;
                    iMode_12_ = 0;
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;
                }
                return;
            }
            else
            {
                // 等待
                if (fDpos_13_ > 100)
                {
                    bIsPushedOut_ = true;
                }
                if (GetPhotoSensorIsTriggered(9) && bIsPushedOut_)
                {
                    bTryGrasp_ = false;
                }
            }
            return;
        }

        // 测试模式 - 推杆按时间运动
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::TestMode_TCPReciprocating_Time)
        {
            return;
        }

        // 测试模式 - 环绕运动 移动中
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::TestMode_EncircleMotion_MoveingToTarget)
        {
            if(isArriveThisPlace(TargetAxisReal_))
            {
                // 到达位置
                llWaitingTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                RobotArmStateData_.ExecuteState_ = ExecuteState::TestMode_EncircleMotion_Arrive;
            }
            return;
        }

        // 测试模式 - 环绕运动 到达位置
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::TestMode_EncircleMotion_Arrive)
        {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
                    - llWaitingTime_ > 500) // 500 毫秒
            {
                llWaitingTime_ = 0;
                if(!qEncircleMotionPose_.empty())
                {
                    TargetPose_ = qEncircleMotionPose_.front();
                    qEncircleMotionPose_.pop();
                    // RY->Pitch  RX->Roll  RZ->Yaw
                    float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                            , (float)(TargetPose_.roll_ / (M_PI / 180))
                                            , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                            , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                    MoveAbs(ZMC_Handle_,6,axis_rt_list,dis, TargetAxisReal_);
                    RobotArmStateData_.ExecuteState_ = ExecuteState::TestMode_EncircleMotion_MoveingToTarget;
                }
                else
                {
                    // SetAxisSpeed(axis_rt_list, 6, 230, 250, 50);
                    SetAxisSpeed(axis_rt_list, 6, 190, 150, 50);
                    RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;
                }
            }
            return;
        }
        if (RobotArmStateData_.ExecuteState_ == ExecuteState::Upload_SingleLoading_ManualMove)
        {
            if (bManualMoveDone_)
            {
                RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;
            }
            return;
        }
    }
        }
    }
}

// 更新设备状态线程
void RobotArm::UpdateThread()
{
    while (!bIsStopUpdate_)
    {
//        long long stime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if( ZMC_Handle_ != nullptr && RobotArmStateData_.RobotArmState_ == RobotArmState::Normal)
        {
            // 获取轴状态
            ForearmConveyorBelt_.Update(ZMC_Handle_);   // 大臂传送带
            BackConveyorBelt_.Update(ZMC_Handle_);    // 背滚传送带
            BigarmConveyorBelt_.Update(ZMC_Handle_);    // 小臂传送带

            //  获取全部轴的运动状态
            //bool vAllAxisEnabled = true;
            for (auto &itAxis : RobotArmStateData_.umapAxisStatus_)
            {
                ZAux_Direct_GetIfIdle(ZMC_Handle_, itAxis.first, &itAxis.second);
                int iAS = 0;
                ZAux_Direct_GetAxisStatus(ZMC_Handle_, itAxis.first, &iAS);
                int iAE = 0;
                ZAux_Direct_GetAxisEnable(ZMC_Handle_, itAxis.first, &iAE);
//                int iAM = 0;
                ZAux_Direct_GetMtype(ZMC_Handle_, itAxis.first, &RobotArmStateData_.umapAxisMtype_[itAxis.first]);

                float axis_speed = 0.0;
                ZAux_Direct_GetSpeed(ZMC_Handle_, itAxis.first, &axis_speed);
                float axis_accel = 0.0;
                ZAux_Direct_GetAccel(ZMC_Handle_, itAxis.first, &axis_accel);
                float axis_decel = 0.0;
                ZAux_Direct_GetDecel(ZMC_Handle_, itAxis.first, &axis_decel);
                float axis_encoder = 0.0;
                ZAux_Direct_GetEncoder(ZMC_Handle_, itAxis.first, &axis_encoder);
//                int32 axis_sdo = 0;
//                ZAux_BusCmd_SDOReadAxis(ZMC_Handle_, itAxis.first, 0x6077, 0,4,&axis_sdo);

                float axis_dpos = 0.0;
                ZAux_Direct_GetDpos(ZMC_Handle_, itAxis.first, &axis_dpos);
                float axis_mpos = 0.0;
                ZAux_Direct_GetMpos(ZMC_Handle_, itAxis.first, &axis_mpos);
                std::stringstream status_data;
                status_data << u8"轴" << itAxis.first << u8" | 状态:" << iAS
                            << u8" | 使能状态:" << iAE
                            << u8" | MType:" << RobotArmStateData_.umapAxisMtype_[itAxis.first]
                            << u8" | DPos:" << axis_dpos
                            << u8" | MPos:" << axis_mpos
                            << u8" | 编码器值:" << QString::number(axis_encoder,'f',8).toStdString();
                // status_data << u8"轴" << itAxis.first << u8" | 状态:" << iAS
                //                << u8" | 使能状态:" << iAE
                //                << u8" | MType:" << RobotArmStateData_.umapAxisMtype_[itAxis.first]
                //                << u8" | DPos:" << axis_dpos
                //                << u8" | MPos:" << axis_mpos
                //                << u8" | 速度" << axis_speed
                //                << u8" | 加速度" << axis_accel
                //                << u8" | 减速度" <<axis_decel;
//                status_data << u8"轴" << itAxis.first << u8" | 状态:" << iAS << u8" | 运动状态:"
//                            << itAxis.second << u8" | 使能状态:" << iAE << u8" | MType:" << RobotArmStateData_.umapAxisMtype_[itAxis.first]
//                            << u8" | 扭矩" << axis_sdo * 0.001;
                            /*<< u8" | 速度" << axis_speed << u8" | 加速度" << axis_accel << u8" | 减速度" <<axis_decel;*/
                RobotArmStateData_.umapAxisStatusString_[itAxis.first] = status_data.str();
                //if (itAxis.first < 6 && iAE < 1)    // 0~5的真实轴
                //{
                //    vAllAxisEnabled = false;
                //}
            }
//            if (!vAllAxisEnabled)
//            {
//                RobotArmStateData_.RobotArmState_ = RobotArmState::NotEnabled;
//            }
//            else if (RobotArmStateData_.RobotArmState_ == RobotArmState::NotEnabled)
//            {
//                RobotArmStateData_.RobotArmState_ = RobotArmState::Normal;
//            }

            int idle;
            float fdpos;
            float fvspeed;

            ZAux_Direct_GetIfIdle(ZMC_Handle_,RobotArmStateData_.iAxisGroupIndex_, &idle);
            RobotArmStateData_.sAxisGroupIndexStatus_ = QString("Idle : %1").arg(idle?"stop":"runing");

            ZAux_Direct_GetDpos(ZMC_Handle_,RobotArmStateData_.iAxisGroupIndex_,&fdpos);
            RobotArmStateData_.sAxisGroupIndexDpos_ = QString("Dpos : %1").arg(fdpos);

            ZAux_Direct_GetVpSpeed(ZMC_Handle_,RobotArmStateData_.iAxisGroupIndex_,&fvspeed);
            RobotArmStateData_.sAxisGroupIndexSpeed_ = QString("Vspeed : %1").arg(fvspeed);

            // 机械臂 五个轴的状态
            float dis_real_list[6] = {0};
            for (int i = 0; i < 6; i++)
            {
                ZAux_Direct_GetDpos(ZMC_Handle_, axis_real_list[i], &dis_real_list[i]);    //获取运动指令发送对位置，若想获取编码器的位置可以获取Mpos的值
            }

            RobotArmStateData_.fL_1_ = dis_real_list[0]; //五个轴的关节角度
            RobotArmStateData_.fL_2_ = dis_real_list[1];
            RobotArmStateData_.fL_3_ = dis_real_list[2];
            RobotArmStateData_.fL_4_ = dis_real_list[3];
            RobotArmStateData_.fL_5_ = dis_real_list[4];

            // T3
            float T3[6]={0,0,0,0,0,0};
            ZAux_Direct_GetTable(ZMC_Handle_, 450, 6, T3);
            RobotArmStateData_.fT3_x_ = T3[0]; //五个轴的T3数据
            RobotArmStateData_.fT3_y_ = T3[1];
            RobotArmStateData_.fT3_z_ = T3[2];
            RobotArmStateData_.fT3_rx_ = T3[3];
            RobotArmStateData_.fT3_ry_ = T3[4];
            RobotArmStateData_.fT3_rz_ = T3[5];
/*
            // 获取虚拟轴对状态
            float dis_rt_list[6] = {0};
            ZAux_Direct_Connreframe(ZMC_Handle_,6,axis_rt_list,121,10,6,axis_real_list);
            ZAux_Direct_SetTable(ZMC_Handle_, 700, 6, dis_real_list);
            ZAux_GetFrameTrans2(ZMC_Handle_, axis_rt_list, 6, 700, 710, 1);
            ZAux_Direct_GetTable(ZMC_Handle_, 710, 6, dis_rt_list);
            RobotArmStateData_.fTcp_x_ = dis_rt_list[0]; //tcp位姿坐标
            RobotArmStateData_.fTcp_y_ = dis_rt_list[1];
            RobotArmStateData_.fTcp_z_ = dis_rt_list[2];
            RobotArmStateData_.fTcp_rx_ = dis_rt_list[3];
            RobotArmStateData_.fTcp_ry_ = dis_rt_list[4];
            RobotArmStateData_.fTcp_rz_ = dis_rt_list[5];
*/
            // 获取推杆位置
            float fEncode = 0;
            ZAux_Direct_GetEncoder(ZMC_Handle_,13,&fEncode);
            ZAux_Direct_GetDpos(ZMC_Handle_,13,&fDpos_13_);
            float fMpos = 0;
            ZAux_Direct_GetMpos(ZMC_Handle_,13,&fMpos);
//            qDebug() << "Encoder:" << fEncode << "   Dpos:" << fDpos_13_ << "   Mpos:" << fMpos;

            // 吸盘运动
            VacuumChuckFunction();


            // 读取输入口 数据
            for (auto &itIoInput : RobotArmStateData_.umapIoInput_)
            {
                uint32 io_data = 0;
                ZAux_Direct_GetIn(ZMC_Handle_,itIoInput.first, &io_data);
                itIoInput.second = io_data;
            }

            // 读取输出口状态
            for (auto &itIoOut : RobotArmStateData_.umapIoOut_)
            {
                if (itIoOut.first == 2000)
                {
                    continue;
                }
                uint32 out_data = 0;
                ZAux_Direct_GetOp(ZMC_Handle_,itIoOut.first, &out_data);
                itIoOut.second = out_data > 0 ? true : false;
            }

            //如果紧急停止按钮被按下，取消轴使能
//            auto itInput_17 = RobotArmStateData_.umapIoInput_.find(17);// 急停按钮
//            if (itInput_17 != RobotArmStateData_.umapIoInput_.end())
//            {
//                if(itInput_17->second == 0)
//                {
//                    iTuboMode_ = 0;
//                    iMode_12_ = 0;
//                    iMode_14_ = 0;
//                    ZAux_Direct_Rapidstop(ZMC_Handle_,2);
//                    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 0, 0);
//                    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 1, 0);
//                    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 2, 0);
//                    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 3, 0);
//                    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 4, 0);
//                    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 13, 0);
//                    ForearmConveyorBelt_.SetEnable(ZMC_Handle_, 0);
//                    BackConveyorBelt_.SetEnable(ZMC_Handle_, 0);
//                    BigarmConveyorBelt_.SetEnable(ZMC_Handle_, 0);
//        //            log_.AddLog(__LINE__, __FILE__, "急停按下", 4, LLOG::TLogLevel::ERROR);
//                }
//            }

            // 卸货时轴十二变速运动
            if(iMode_12_ == 1)
            {
                if (fDpos_13_ < 10 && RobotArmStateData_.umapIoInput_[9] > 0)
                {
                    BigarmConveyorBelt_.SetSpeed(ZMC_Handle_, 400);
                    BigarmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
                }
                else if (fDpos_13_ < 220)
                {
                    BigarmConveyorBelt_.Stop(ZMC_Handle_);
                }
                else
                {
                    BigarmConveyorBelt_.SetSpeed(ZMC_Handle_, 250);
                    BigarmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
                }
            }
            else if (iMode_12_ == 5)
            {
                if (RobotArmStateData_.umapIoInput_[9] > 0)
                {
                    BigarmConveyorBelt_.SetSpeed(ZMC_Handle_, 400);
                    BigarmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
                }
                else if (fDpos_13_ < -370)
                {
                    BigarmConveyorBelt_.Stop(ZMC_Handle_);
                }
                else
                {
                    BigarmConveyorBelt_.SetSpeed(ZMC_Handle_, 250);
                    BigarmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
                }
            }
//            else if (iMode_12_ == 0)
//            {
//                BigarmConveyorBelt_.Stop(ZMC_Handle_);
//            }

            // 大臂转动
            if(iMode_14_ == 1)  // 卸货 反转 -1
            {
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 110.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, -1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, -1);
            }
//            else if (iMode_14_ == 0)    // 停止转动
//            {
//                ForearmConveyorBelt_.Stop(ZMC_Handle_);
//            }

            // 碰撞检测传感器
//            auto itInput_15 = RobotArmStateData_.umapIoInput_.find(15);
//            auto itInput_6 = RobotArmStateData_.umapIoInput_.find(6);
//            auto itInput_7 = RobotArmStateData_.umapIoInput_.find(7);
//            if(itInput_15 != RobotArmStateData_.umapIoInput_.end())
//            {
//                if(itInput_15->second > 0) // 激光扫描仪是否工作
//                {
//                    if (itInput_6 != RobotArmStateData_.umapIoInput_.end() || itInput_7 != RobotArmStateData_.umapIoInput_.end())
//                    {
//                        if(itInput_6->second > 0 || itInput_7->second > 0)
//                        {
//                            qDebug()<< "it_input_6" << itInput_6->second << QString("%1").arg(itInput_6->second ? "6口设备检测达到碰撞区域" : "检查7口设备");
//                            qDebug()<< "it_input_7" << itInput_7->second << QString("%1").arg(itInput_7->second ? "7口设备检测达到碰撞区域" : "检查6口设备");
//                            ZAux_Direct_Rapidstop(ZMC_Handle_,2);
//                            iTuboMode_ = 0;
//                            iMode_12_ = 0;
//                            iMode_14_ = 0;
//                            BigarmConveyorBelt_.Stop(ZMC_Handle_);
//                            ForearmConveyorBelt_.Stop(ZMC_Handle_);
//                            BackConveyorBelt_.Stop(ZMC_Handle_);
//                        }
//                    }
//                }
//            }

            // 获取测距数据
            RobotArmStateData_.dLaserLeftDist_ = 0;
            RobotArmStateData_.dLaserDownDist_ = 0;
            RobotArmStateData_.dLaserShortDist_ = 0;
            // 向左
//            float DaValue_0 = 0.0;
//            // DaValue_0*10/4095是读到的电压值，然后*984.4+88.2为对应的距离值单位是mm
//            ZAux_Direct_GetAD(ZMC_Handle_, 0, &DaValue_0);

//            // RobotArmStateData_.dLaserLeftDist_ = 1100.0;
//            RobotArmStateData_.dLaserLeftDist_ = (DaValue_0 * 10 / 4095 * 99 + 98.586);
//            RobotArmStateData_.dLaserLeftDist_ -= 119.0;    // 减去小臂边缘到测距仪的位置
//            if (RobotArmStateData_.dLaserLeftDist_ < 0.0)
//            {
//                RobotArmStateData_.dLaserLeftDist_ = 0.0;
//            }
            // 前短距
//            RobotArmStateData_.dLaserShortDist_ = LaserShortDist_.GetDistance() + 144.0-5.5;
//            RobotArmStateData_.dLaserShortDist_ -= 447.0;   // 距离末端边缘有10厘米
//            if (RobotArmStateData_.dLaserShortDist_ < 0.0)
//            {
//                RobotArmStateData_.dLaserShortDist_ = 0.0;
//            }

            // 前长距
            RobotArmStateData_.dLaserLongDist_ = FrontLaserDist_.GetDistance() - 323;
//            if (RobotArmStateData_.dLaserLongDist_ > 0)
//            {
//                RobotArmStateData_.dLaserLongDist_ -= 380;
//            }
//            RobotArmStateData_.dLaserLongDist_ = 1189;  // 理想传感器输出值
//            RobotArmStateData_.dLaserLongDist_ -= 489;  // 安装位置到TCP原点的位置
//            RobotArmStateData_.dLaserLongDist_ += 70;  // TCP原点到边缘的位置
//            RobotArmStateData_.dLaserLongDist_ += 100;
        //        if (RobotArmStateData_.dLaserLongDist_ < 0)
        //        {
        //            RobotArmStateData_.dLaserLongDist_ = 0;
        //        }

            // 向下
//            RobotArmStateData_.dLaserDownDist_ = LaserDownDist_.GetDistance() * 10; //厘米转毫米
//            RobotArmStateData_.dLaserDownDist_ -= 100.0;   // 距离末端边缘有10厘米
//            if (RobotArmStateData_.dLaserDownDist_ < 0.0)
//            {
//                RobotArmStateData_.dLaserDownDist_ = 0.0;
//            }
//            RobotArmStateData_.dLaserDownDist_ = 1535; // 1188.5; //1150.0;  385 + 1150
//            RobotArmStateData_.dLaserDownDist_ -= 130;
//            RobotArmStateData_.dLaserDownDist_ = 1725; // 1660; // 1188.5; //1150.0;  385 + 1150
//            RobotArmStateData_.dLaserDownDist_ -= 210;
//            RobotArmStateData_.dLaserDownDist_ -= 197;
//            RobotArmStateData_.dLaserDownDist_ -= 13;

            RobotArmStateData_.iTestModeRunCount_ = iTestModeRunCount_;
            RobotArmStateData_.iTestModeEncircleMotionRunCount_ = qEncircleMotionPose_.size();
        //        qDebug() << u8"短距离测距激光的数据：" << RobotArmStateData_.dLaserShortDist_;
        //        qDebug() << u8"长距离测距激光的数据：" << RobotArmStateData_.dLaserLongDist_;
        //        qDebug() << u8"左侧测距激光的数据：" << RobotArmStateData_.dLaserLeftDist_;
        //        qDebug() << u8"向下测距激光的数据：" << RobotArmStateData_.dLaserDownDist_;

            // 更新数据
            pStateCallback_(RobotArmStateData_);

        }
//        qDebug() << "使用时间:" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() -  stime;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}

void RobotArm::StopUpdateThread()
{
    bIsStopUpdate_ = true;
    if (thUpdateThread_.joinable())
    {
        thUpdateThread_.join();
    }
}

void RobotArm::ComponentInit()
{
    //ZMC系列认为OFF时碰到了原点信号（常闭）
    ZAux_Direct_SetInvertIn(ZMC_Handle_, 9, 0);

    // 大臂传送带
    ForearmConveyorBelt_.iAxisID_ = 14;
//    ForearmConveyorBelt_.SetType(ZMC_Handle_, 66);
    ForearmConveyorBelt_.SetUnits(ZMC_Handle_, 552731);
    ForearmConveyorBelt_.SetAccel(ZMC_Handle_, 200);
    ForearmConveyorBelt_.SetDecel(ZMC_Handle_, 200);
    ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 250);
    ForearmConveyorBelt_.Update(ZMC_Handle_);

    // 小臂传送带
    BigarmConveyorBelt_.iAxisID_ = 12;
//    BigarmConveyorBelt_.SetType(ZMC_Handle_, 66);
    BigarmConveyorBelt_.SetUnits(ZMC_Handle_, 552731);
    BigarmConveyorBelt_.SetAccel(ZMC_Handle_, 200);
    BigarmConveyorBelt_.SetDecel(ZMC_Handle_, 200);
    BigarmConveyorBelt_.SetSpeed(ZMC_Handle_, 250);
    BigarmConveyorBelt_.Update(ZMC_Handle_);

    // 小臂 推杆
    BigarmSuctionCupBracket_.iAxisID_ = 13;
    BigarmSuctionCupBracket_.SetType(ZMC_Handle_, 65);
    BigarmSuctionCupBracket_.SetUnits(ZMC_Handle_, 552730);
    BigarmSuctionCupBracket_.SetLspeed(ZMC_Handle_, 100);
    BigarmSuctionCupBracket_.SetSpeed(ZMC_Handle_, 300);
    BigarmSuctionCupBracket_.SetAccel(ZMC_Handle_, 200);
    BigarmSuctionCupBracket_.SetDecel(ZMC_Handle_, 600);
    BigarmSuctionCupBracket_.SetForceSpeed(ZMC_Handle_, 300);
    BigarmSuctionCupBracket_.SetStartMoveSpeed(ZMC_Handle_, 100);
    BigarmSuctionCupBracket_.SetEndMoveSpeed(ZMC_Handle_, 100);
    BigarmSuctionCupBracket_.SetCreep(ZMC_Handle_, 150);
    BigarmSuctionCupBracket_.SetMerge(ZMC_Handle_, 1);
    BigarmSuctionCupBracket_.SetDatumIn(ZMC_Handle_, 9);
    BigarmSuctionCupBracket_.Update(ZMC_Handle_);


    // 背部传送带
    BackConveyorBelt_.iAxisID_ = 15;
//    BigarmConveyorBelt_.SetType(ZMC_Handle_, 66);
    BackConveyorBelt_.SetUnits(ZMC_Handle_, 125424);
    BackConveyorBelt_.SetAccel(ZMC_Handle_, 200);
    BackConveyorBelt_.SetDecel(ZMC_Handle_, 200);
    BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
    BackConveyorBelt_.Update(ZMC_Handle_);
}

// 接受来自界面的控制数据
void RobotArm::ControlData(const RobotArmControl& control_data)
{
    switch (control_data.ControlType_)
    {
        case RobotArmControl::ControlType::AxisGroupIndex:
        {
            RobotArmStateData_.iAxisGroupIndex_ = control_data.iAxisGroupIndex_;
            break;
        }
        case RobotArmControl::ControlType::InitIoOut:
        {
            // 初始化Io out，的状态和数量
            for (auto itIoData : control_data.umapIoData_)
            {
                RobotArmStateData_.umapIoOut_[itIoData.first] = false;
            }
            break;
        }
        case RobotArmControl::ControlType::IoOut:
        {
            // 点击了out按钮
            RobotArmStateData_.umapIoOut_[control_data.IoID_] = control_data.IoState_;
            IoOut(control_data.IoID_, control_data.IoState_);
            break;
        }
        case RobotArmControl::ControlType::InitIoInput:
        {
            // 初始化Io input，的状态和数量
            for (auto itIoData : control_data.umapIoData_)
            {
                RobotArmStateData_.umapIoInput_[itIoData.first] = 1;
            }
            break;
        }
        case RobotArmControl::ControlType::RunButton:
        {
            RunSingleAxis(control_data);
            break;
        }
        case RobotArmControl::ControlType::StopButton:
        {
            StopSingleAxis(control_data);
            break;
        }
        case RobotArmControl::ControlType::SingleAxisReset:
        {
            SingleAxisReset(control_data);
            break;
        }
        case RobotArmControl::ControlType::SingleAxisResetCoder:
        {
            SingleAxisResetCoder(control_data);
            break;
        }
        case RobotArmControl::ControlType::AxisEnableSignal:
        {
            if (ZMC_Handle_ != nullptr)
            {
                if (ZAux_BasDown(ZMC_Handle_, control_data.strData_.c_str(), 0) == 0)
                {
//                    ZAux_Direct_Connreframe(ZMC_Handle_, 6, axis_rt_list, 121, 10, 6, axis_real_list); //建立正解链接

                    if (RobotArmStateData_.ExecuteState_ != ExecuteState::Idle)
                    {
                        RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;
                    }
                }
            }
            break;
        }
        case RobotArmControl::ControlType::StopAllAxis:
        {
            StopAllAxis();
            break;
        }
        case RobotArmControl::ControlType::TcpHoming:
        {
            iTuboMode_ = 0;
            iMode_12_ = 0;
            TcpHoming();
            break;
        }
        case RobotArmControl::ControlType::TcpPush:
        {
            TcpPush();
            break;
        }
        case RobotArmControl::ControlType::TcpSingleRun:
        {
            iTuboMode_ = 1;
            iMode_12_ = 1;
            tcp_single_run();
            break;
        }
        case RobotArmControl::ControlType::RtRun:
        {
            if (ZMC_Handle_ != nullptr)
            {
                RobotArmStateData_.iRobotStatus_ = 2;
//                ZAux_Direct_Rapidstop(ZMC_Handle_,2);
                float dis[6] = {(float)control_data.vecValue_[0], (float)control_data.vecValue_[1], (float)control_data.vecValue_[2],
                                  (float)control_data.vecValue_[3], (float)control_data.vecValue_[4], (float)control_data.vecValue_[5]};
                ZAux_Direct_MoveAbs(ZMC_Handle_, 6,axis_rt_list,dis);
//                if (RobotArmStateData_.umapAxisMtype_[6] != 33)
//                {
//                    qDebug() << "建立逆解模式 : " << ZAux_Direct_Connframe(ZMC_Handle_,6,axis_real_list,121,10,6,axis_rt_list); //建立逆解模式
//                }
//                MoveAbs(ZMC_Handle_,6,axis_rt_list,dis, TargetAxisReal_);
            }
            break;
        }
        case RobotArmControl::ControlType::BtRt:
        {
            qDebug() << "建立逆解模式 : " << ZAux_Direct_Connframe(ZMC_Handle_,6,axis_real_list,121,10,6,axis_rt_list); //建立逆解模式
            break;
        }
        case RobotArmControl::ControlType::BtReal:
        {
            qDebug() << "创建正解 : " << ZAux_Direct_Connreframe(ZMC_Handle_, 6, axis_rt_list, 121, 10, 6, axis_real_list); //建立正解链接
            break;
        }
        case RobotArmControl::ControlType::RealRun:
        {
            if (ZMC_Handle_ != nullptr)
            {
//                if (RobotArmStateData_.umapAxisMtype_[6] != 34)
//                {
//                    qDebug() << "创建正解 : " << ZAux_Direct_Connreframe(ZMC_Handle_, 6, axis_rt_list, 121, 10, 6, axis_real_list); //建立正解链接
//                }
                float dis[6]={0};
                dis[0] = control_data.vecValue_[0];
                dis[1] = control_data.vecValue_[1];
                dis[2] = control_data.vecValue_[2];
                dis[3] = control_data.vecValue_[3];
                dis[4] = control_data.vecValue_[4];
                dis[5] = control_data.vecValue_[5];

                ZAux_Direct_MoveAbs(ZMC_Handle_, 6, axis_real_list, dis);
            }
            break;
        }
        case RobotArmControl::ControlType::Rapidstop:
        {
            if (ZMC_Handle_ != nullptr)
            {
                ZAux_Direct_Rapidstop(ZMC_Handle_,2);
                qDebug() << "清缓存";
            }
            break;
        }
        case RobotArmControl::ControlType::GetTcp:
        {
            if (ZMC_Handle_ != nullptr)
            {
                GetTcpData();
            }
            break;
        }
        case RobotArmControl::ControlType::SaveTcpAndT3:
        {
            GetTcpData();
            std::stringstream tcp_log_data;
            tcp_log_data << u8"TCP 位置 :" << RobotArmStateData_.fTcp_x_
                     << "," << RobotArmStateData_.fTcp_y_
                     << "," << RobotArmStateData_.fTcp_z_
                     << "," << RobotArmStateData_.fTcp_rx_
                     << "," << RobotArmStateData_.fTcp_ry_
                     << "," << RobotArmStateData_.fTcp_rz_;
            log_.AddLog(__LINE__, __FILE__, tcp_log_data.str().c_str(), tcp_log_data.str().size(), LLOG::TLogLevel::DEBUG);

            float T3[6]={0,0,0,0,0,0};
            ZAux_Direct_GetTable(ZMC_Handle_, 450, 6, T3);

            std::stringstream t3_log_data;
            t3_log_data << u8"T3 位置 :" << T3[0]
                     << "," << T3[1]
                     << "," << T3[2]
                     << "," << T3[3]
                     << "," << T3[4]
                     << "," << T3[5];
            log_.AddLog(__LINE__, __FILE__, t3_log_data.str().c_str(), t3_log_data.str().size(), LLOG::TLogLevel::DEBUG);
            break;
        }
        case RobotArmControl::ControlType::ConnectArm:
        {
            if(ZMC_Handle_ == nullptr && RobotArmStateData_.RobotArmState_ < RobotArmState::Normal)  // 尝试连接服务器
            {
                int32 iresult = ZAux_OpenEth(ip_,&ZMC_Handle_);
                if(0 == iresult)
                {
                    std::string log_data("连接控制器成功!");
                    log_.AddLog(__LINE__, __FILE__, log_data.c_str(), log_data.size(), LLOG::TLogLevel::DEBUG);
                    ComponentInit();    // 组件模块初始化
                    RobotArmStateData_.RobotArmState_ = RobotArmState::Normal;
                    pStateCallback_(RobotArmStateData_);
                    // 创建线程获取 机械臂数据
                    bIsStopUpdate_ = false;
                    thUpdateThread_ = std::thread(&RobotArm::UpdateThread, this);
                }
            }
            break;
        }
        case RobotArmControl::ControlType::TestMode_StartRunCount:
        {
            bTryGrasp_ = false;
            iTestModeRunCount_ = (int)control_data.vecValue_[0];
            if (control_data.vecValue_[1] > 0)
            {
                bTestModeEnablementConveyor_ = true;
            }
            else
            {
                bTestModeEnablementConveyor_ = false;
            }
            RobotArmStateData_.ExecuteState_ = ExecuteState::TestMode_TCPReciprocating_Count;
            break;
        }
        case RobotArmControl::ControlType::TestMode_StopRunCount:
        {
            iTestModeRunCount_ = 0;
            break;
        }
        case RobotArmControl::ControlType::TestMode_StartRunConveyor:
        {
            if (control_data.IoState_)
            {   // 上货方向
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 200.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, 1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 500.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, 1);
            }
            else
            {   // 卸货方向
                ForearmConveyorBelt_.SetSpeed(ZMC_Handle_, 110.0);
                ForearmConveyorBelt_.ContinueRun(ZMC_Handle_, -1);

                BackConveyorBelt_.SetSpeed(ZMC_Handle_, 550.0);
                BackConveyorBelt_.ContinueRun(ZMC_Handle_, -1);
            }
            break;
        }
        case RobotArmControl::ControlType::TestMode_StopRunConveyor:
        {
            ForearmConveyorBelt_.Stop(ZMC_Handle_);
            BackConveyorBelt_.Stop(ZMC_Handle_);
            break;
        }
        case RobotArmControl::ControlType::TestMode_StartEncircleMotion:
        {   // 开始环绕运动
            // 判断当前是否适合 进行运动
            if ((ZMC_Handle_ != nullptr)
                && (RobotArmStateData_.RobotArmState_ == RobotArmState::Normal)
                && (RobotArmStateData_.ExecuteState_ == ExecuteState::Idle))
            {
                // 获取更新当前位置
                GetTcpData();
                // 更改逆解速度 为较低速度
                SetAxisSpeed(axis_rt_list, 6, 5, 3, 3);
                // 根据当前位置 算出需要移动的点
//                Common::Pose temp_pose( RobotArmStateData_.fTcp_x_
//                                       ,RobotArmStateData_.fTcp_y_
//                                       ,RobotArmStateData_.fTcp_z_, 0, 0, 0);
                double tcp_x = RobotArmStateData_.fTcp_x_;
                double tcp_y = RobotArmStateData_.fTcp_y_;
                double tcp_z = RobotArmStateData_.fTcp_z_;
                // 15,  38, 35.5, 25,  15, 15
                // -15,-35,-61.5,-25,-15,-15
                for (int temp_pitch = -14; temp_pitch < 14; temp_pitch+=3)
                {
                    for (int temp_yaw = -24; temp_yaw < 24; temp_yaw+=3)
                    {
                        Common::Pose temp_pose(tcp_x, tcp_y ,tcp_z
                                               , (double)temp_pitch * (M_PI / 180)
                                               , 0 * (M_PI / 180)
                                               , (double)temp_yaw * (M_PI / 180));
                        // 查询当前位置是否有解
                        if (IsEffectivePose(temp_pose))
                        {
                            // 加入队列
                            qEncircleMotionPose_.push(temp_pose);
                        }
                    }
                }
                if(!qEncircleMotionPose_.empty())
                {
                    TargetPose_ = qEncircleMotionPose_.front();
                    qEncircleMotionPose_.pop();
                    // RY->Pitch  RX->Roll  RZ->Yaw
                    float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                            , (float)(TargetPose_.roll_ / (M_PI / 180))
                                            , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                            , (float)(TargetPose_.yaw_ / (M_PI / 180))};
                    MoveAbs(ZMC_Handle_,6,axis_rt_list,dis, TargetAxisReal_);
                    RobotArmStateData_.ExecuteState_ = ExecuteState::TestMode_EncircleMotion_MoveingToTarget;
                }
            }
            break;
        }
        case RobotArmControl::ControlType::TestMode_StopEncircleMotion:
        {   // 结束 环绕运动
            break;
        }
        default :
        {
            break;
        }
    }
}


// 相应 界面点击 IO口的控制
void RobotArm::IoOut(int id, bool state)
{
    if(nullptr == ZMC_Handle_)
    {
        qDebug() << "先链接控制器";
        return;
    }
    if (id == 2000)
    {
        if(state)
        {
           uint8 iostate = 15;
           ZAux_Modbus_Set0x(ZMC_Handle_, id, 4, &iostate);	//MODBUS_BIT(20000)映射到控制输出口 ，打开输出0-3 二进制15 1111

        }
        else
        {
          uint8 iostate = 0;
          ZAux_Modbus_Set0x(ZMC_Handle_, id, 4, &iostate);// close

        }
    }
    else
    {
        ZAux_Direct_SetOp(ZMC_Handle_, id, state);
    }
}

Eigen::Matrix4d RobotArm::PoseToMatrix4d(std::vector<float> &xyzrpy)
{
  Eigen::Matrix4d RT;
  double rx = xyzrpy[3]*M_PI/180;
  double ry = xyzrpy[4]*M_PI/180;
  double rz = xyzrpy[5]*M_PI/180;

  RT << cos(rz)*cos(ry), cos(rz)*sin(ry)*sin(rx)-sin(rz)*cos(rx), cos(rz)*sin(ry)*cos(rx)+sin(rz)*sin(rx), xyzrpy[0],
        sin(rz)*cos(ry), sin(rz)*sin(ry)*sin(rx)+cos(rz)*cos(rx), sin(rz)*sin(ry)*cos(rx)-cos(rz)*sin(rx), xyzrpy[1],
               -sin(ry),                         cos(ry)*sin(rx),                         cos(ry)*cos(rx), xyzrpy[2],
                      0,                                       0,                                       0,         1;

  return RT;
}


int RobotArm::GetRefinedPose(Common::Pose &box_pose, double y_l, double y_r)
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d source_pose;
    source_pose[0] = box_pose.x_;
    source_pose[1] = box_pose.y_;
    source_pose[2] = box_pose.z_;
    source_pose[3] = box_pose.roll_ / (M_PI / 180);
    source_pose[4] = box_pose.pitch_ / (M_PI / 180);
    source_pose[5] = box_pose.yaw_ / (M_PI / 180);


    int label_id = 0;
    const double tcp_top_z_limited_    = 1600.0;       // the best threshold   (get by teaching)
    const double tcp_bottom_z_limited_ = 400; // 60.0;          // the best threshold   (get by teaching)
    const double tcp_left_y_limited_   = y_l; // 865.0;        // the best threshold   (get by teaching)
    const double tcp_right_y_limited_  = y_r; // -767;       // the best threshold   (get by teaching)
    const double delta_ry_for_top_     = -7.0;         // the best threshold   (get by teaching)
    const double delta_ry_for_bottom_  = 7;         // the best threshold   (get by teaching)


    Vector6d target_pose = source_pose;
    // to refine pose(vec6d)
    // is top & left
    if (source_pose[1] > tcp_left_y_limited_ && (source_pose[2] > tcp_top_z_limited_))
    {
        label_id = 1;                                       //to label 1
        target_pose[1] = tcp_left_y_limited_;
        target_pose[4] = delta_ry_for_top_;
    }
    // is top & right
    else if(source_pose[1] < tcp_right_y_limited_ && (source_pose[2] > tcp_top_z_limited_))
    {
        label_id = 2;                                       //to label 2
        target_pose[1] = tcp_right_y_limited_;
        target_pose[4] = delta_ry_for_top_;
    }
    // is bottom & left
    else if(source_pose[1] > tcp_left_y_limited_ && (source_pose[2] < tcp_bottom_z_limited_))
    {
      label_id = 3;                                       //to label 3
      target_pose[1] = tcp_left_y_limited_;
      target_pose[2] += 5;
      target_pose[4] = delta_ry_for_bottom_;
    }

    // is bottom & right
    else if(source_pose[1] < tcp_right_y_limited_ && (source_pose[2] < tcp_bottom_z_limited_))
    {
      label_id = 4;                                       //to label 4
      target_pose[1] = tcp_right_y_limited_;
      target_pose[2] += 5;
      target_pose[4] = delta_ry_for_bottom_;
    }

    // is top
    else if(source_pose[2] > tcp_top_z_limited_)
    {
      label_id = 5;                                        //to label 5
      target_pose[4] = delta_ry_for_top_;
    }

    // is left
    else if(source_pose[1] > tcp_left_y_limited_)
    {
      label_id = 6;                                       //to label 6
      target_pose[1] = tcp_left_y_limited_;
    }

    // is right
    else if(source_pose[1] < tcp_right_y_limited_)
    {
      label_id = 7;                                        //to label 7
      target_pose[1] = tcp_right_y_limited_;
    }
    // is bottom
    else if(source_pose[2] < tcp_bottom_z_limited_)
    {
        label_id = 8;                                         //to label 8
        target_pose[2] += 5;
        target_pose[4] = delta_ry_for_bottom_;
    }
    // is normal
    else{
      label_id = 9;                                         //to label 9
    }


    box_pose.SetPose(target_pose[0], target_pose[1], target_pose[2]
                     , target_pose[4] * (M_PI / 180)
                     , target_pose[3] * (M_PI / 180)
                     , target_pose[5] * (M_PI / 180));
    return label_id;
}

// 抓取箱子
bool RobotArm::GraspingBox(const std::vector<Common::Pose> &box_pose, int pose_id)
{
    if (RobotArmStateData_.RobotArmState_ == RobotArmState::Normal)
    {
        if(ZMC_Handle_ == nullptr)
        {
            qDebug() << u8"警告  " << "未链接到控制器";
            return false;
        }
        GetTcpData();
//        double container_y_left_a = RobotArmStateData_.fTcp_y_
//                                    + RobotArmStateData_.dLaserLeftDist_ + (555 / 2);

//        double container_y_right_a = RobotArmStateData_.fTcp_y_ + -((2290 - (555 / 2))
//                                - RobotArmStateData_.dLaserLeftDist_);
//        qDebug() << "container_y_right_a:" << container_y_right_a;
//        qDebug() << "container_y_left_a:" << container_y_left_a;

        std::stringstream log_data0;
        log_data0 << u8"抓取位置点数" << box_pose.size() ;
        log_.AddLog(__LINE__, __FILE__, log_data0.str().c_str(), log_data0.str().size(), LLOG::TLogLevel::DEBUG);

        // 加载箱体数据
        int box_index = 1;
        for (auto it_box_pose : box_pose)
        {
            // 相机与T3的标定矩阵
            Eigen::Matrix4d C_2_T3_Matrix;
            C_2_T3_Matrix  << 0.0233331,    -0.0870594, 0.99593,    3281.32 / 1000,
                              -0.0245225,   -0.995951,  -0.0864867, -604.529 / 1000,
                              0.999427,     -0.0224046, -0.0253735, 29.77 / 1000,
                              0,            0,          0,          1;
            it_box_pose.TransformPose(C_2_T3_Matrix);

            std::vector<float> vecT3;
            vecT3.push_back(RobotArmStateData_.fT3_x_ / 1000);
            vecT3.push_back(RobotArmStateData_.fT3_y_ / 1000);
            vecT3.push_back(RobotArmStateData_.fT3_z_ / 1000);
            vecT3.push_back(-RobotArmStateData_.fT3_rx_);
            vecT3.push_back(RobotArmStateData_.fT3_ry_);
            vecT3.push_back(RobotArmStateData_.fT3_rz_);
            Eigen::Matrix4d T3_2_Base_Matrix = PoseToMatrix4d(vecT3);

            it_box_pose.TransformPose(T3_2_Base_Matrix);
//            if (box_index % 6 == 0)
//            {
//                if ((it_box_pose.y_* 1000) < -820)
//                {
//                    it_box_pose.SetPose((it_box_pose.x_ * 1000) + 16, -820, (it_box_pose.z_* 1000) + 30, 0, 0, 0);
//                }
//                else
//                {
//                    it_box_pose.SetPose((it_box_pose.x_ * 1000) + 16, it_box_pose.y_* 1000, (it_box_pose.z_* 1000) + 30, 0, 0, 0);
//                }
//            }
//            else
//            {
//                it_box_pose.SetPose((it_box_pose.x_ * 1000) + 16, it_box_pose.y_* 1000, (it_box_pose.z_* 1000) + 30, 0, 0, 0);
//            }
            //-12 * (M_PI / 180)
            it_box_pose.SetPose((it_box_pose.x_ * 1000) + 10, (it_box_pose.y_* 1000), (it_box_pose.z_* 1000) + 55, 0, 0, 0);

            int iLabel = GetRefinedPose(it_box_pose, RobotArmStateData_.dLeftLimitPoint_, RobotArmStateData_.dRightLimitPoint_);
            if((iLabel == 2)
             || (iLabel == 7)
             || (iLabel == 4))
            {
                it_box_pose.z_ += 5; // 由于吸盘架变形出现翻滚，所以最右侧的全部加一个高度。
            }
            std::stringstream log_data;
            log_data << u8"抓取位置点" << box_index << " x:" << it_box_pose.x_ << u8" y:" << it_box_pose.y_ \
                     << u8" z:" << it_box_pose.z_
                     << u8" pitch_:" << it_box_pose.pitch_ / (M_PI / 180)
                     << u8" roll:" << it_box_pose.roll_ / (M_PI / 180)
                     << u8" yaw:" << it_box_pose.yaw_ / (M_PI / 180);
            log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);

            if (it_box_pose.x_ >= 5860) // 防止识别到第二面的数据
            {
                continue;
            }

            if ((it_box_pose.z_ < 700) && pose_id == 1) //700
            {
                break;
            }
            Common::Box temp_box(box_length, box_width, box_height);
            temp_box.OperationPose_ = it_box_pose;
            temp_box.iLabel_ = iLabel;
            qUnloadBox_.push(temp_box);

            box_index++;
        }
        RobotArmStateData_.iRobotStatus_ = 2;

        if (!qUnloadBox_.empty())
        {
            ExecutedBox_ = qUnloadBox_.front();
            qUnloadBox_.pop();
            ExecutedBoxLabel_ = ExecutedBox_.iLabel_;
            TargetPose_ = ExecutedBox_.OperationPose_;
            first_z = TargetPose_.z_;
            bDepthOffset_ = false;
            // RY->Pitch  RX->Roll  RZ->Yaw
            float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                    , (float)(TargetPose_.roll_ / (M_PI / 180))
                                    , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                    , (float)(TargetPose_.yaw_ / (M_PI / 180))};
            MoveAbs(ZMC_Handle_,6,axis_rt_list,dis, TargetAxisReal_);
            RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_MoveingtoTarget;
        }
        else
        {   // 一个都没有 视为完成全部抓取
            RobotArmStateData_.ExecuteState_ = ExecuteState::Unload_Arrive;
            pStateCallback_(RobotArmStateData_);
            RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;
        }
        return true;
    }
    return false;
}

bool RobotArm::UploadingBox(const Common::Container &container
                            , const Common::Box &current_box)
{
    GetTcpData();

    // 获取上/下/左的测距 算出右下角
    double tcp_width = 555.0; // 小臂宽度 mm
    double tcp_thickness = 350.0; // 小臂厚度 mm
    double safe_distance = 50.0;  // 与墙面的安全距离

    // 集装箱Y轴 左侧的机械臂坐标系位置
    double container_y_left_a = RobotArmStateData_.fTcp_y_ \
                                + RobotArmStateData_.dLaserLeftDist_ + (tcp_width / 2);
    // 集装箱Y轴 右侧的机械臂坐标系位置
    double container_y_right_a = RobotArmStateData_.fTcp_y_ + -((container.dInsideWidth_ - (tcp_width / 2)) \
                            - RobotArmStateData_.dLaserLeftDist_);
    // 集装箱z轴 上方的机械臂坐标系位置
    double container_z_top_a = RobotArmStateData_.fTcp_z_ + ((container.dInsideHeight_ - 210) \
                            - RobotArmStateData_.dLaserDownDist_);
    // 集装箱z轴 下方的机械臂坐标系位置
    double container_z_bottom_a = RobotArmStateData_.fTcp_z_ - 210 - RobotArmStateData_.dLaserDownDist_;
    // 集装箱x轴 正面的机械臂坐标系位置
    double container_x_front_a = RobotArmStateData_.fTcp_x_ + (RobotArmStateData_.dLaserLongDist_ - current_box.Length - 100);

    qDebug() << "计算得到的左侧的机械臂坐标系位置container_y_left_a：" << container_y_left_a;
    qDebug() << "计算得到的右侧的机械臂坐标系位置container_y_right_a：" << container_y_right_a;
    qDebug() << "计算得到的上方的机械臂坐标系位置container_z_top_a：" << container_z_top_a;
    qDebug() << "计算得到的下方的机械臂坐标系位置container_z_bottom_a：" << container_z_bottom_a;
    qDebug() << "计算得到的正面的机械臂坐标系位置container_x_front_a：" << container_x_front_a;

    int iBoxIndex = 0;
    int iRowIndex = 0;

    while (true)
    {
        if (iRowIndex == 14)
        {
            qDebug() << iRowIndex;
        }
        bool LineFeed = false;
        // y轴
        Common::Box temp_box(current_box.Length, current_box.Width, current_box.Height);

        if ( ((iBoxIndex * current_box.Width) + container_y_right_a + (tcp_width / 2))
             - (tcp_width - current_box.Width + (tcp_width / 2)) < container_y_right_a) // 装载位置 - (需要往右拨的距离 + 臂的一半宽度)  要是超出边界就认为是右侧，并需要拨正
        {   // 右侧箱子
            if (((iBoxIndex * current_box.Width) + container_y_right_a + (tcp_width / 2))
                    - (tcp_width - current_box.Width + (tcp_width / 2)) - container_y_right_a < safe_distance)
            {
               temp_box.LoadPose_.y_ = (iBoxIndex * current_box.Width) + container_y_right_a + (tcp_width / 2) + safe_distance;
            }
            else
            {
                temp_box.LoadPose_.y_ = (iBoxIndex * current_box.Width) + container_y_right_a + (tcp_width / 2);
            }
            temp_box.BoxPoseType_ = Common::BoxPoseType::Right;
            temp_box.bNeedCorrected_ = true;
            iBoxIndex++;
        }
        else if ( (((iBoxIndex + 2) * current_box.Width) + container_y_right_a) > container_y_left_a)
        {   // 当前是 最后单行 最左侧的箱子
//            temp_box.LoadPose_.y_ -= (temp_box.LoadPose_.y_ + (tcp_width / 2) + tcp_baffle_width) - container_y_left_a;
            if ( ((iBoxIndex * current_box.Width) + container_y_right_a + (tcp_width / 2) + safe_distance) < container_y_left_a)
            {
                temp_box.LoadPose_.y_ = (iBoxIndex * current_box.Width) + container_y_right_a + (tcp_width / 2);
            }
            else
            {
                temp_box.LoadPose_.y_ = container_y_left_a - safe_distance - (tcp_width / 2);
            }
            temp_box.BoxPoseType_ = Common::BoxPoseType::Left;
            temp_box.bNeedCorrected_ = false;
            iBoxIndex = 0;
            LineFeed = true;
        }
        else
        {   // 中间箱子
            temp_box.LoadPose_.y_ = (iBoxIndex * current_box.Width) + container_y_right_a + (tcp_width / 2);
            temp_box.BoxPoseType_ = Common::BoxPoseType::Middle;
            temp_box.bNeedCorrected_ = false;
            iBoxIndex++;
        }

        // x轴
        temp_box.LoadPose_.x_ = container_x_front_a;

        // z轴
        if (iRowIndex >= 2)
        {
//            temp_box.LoadPose_.z_ = (100 + ((iRowIndex - 2) * current_box.Height));
            temp_box.LoadPose_.z_ = (container_z_bottom_a + ((iRowIndex + 1) * current_box.Height)) - (current_box.Height / 2);// ((current_box.Height / 3)*2);
            temp_box.LoadPose_.z_ += 50;
        }
        else
        {
            // 底边位置 + N个箱子高度 - 1个箱子的1/2高度;  // 120是
            temp_box.LoadPose_.z_ = (container_z_bottom_a + ((iRowIndex + 1) * current_box.Height)) - (current_box.Height / 2);// ((current_box.Height / 3)*2);
//            temp_box.LoadPose_.z_ = (container_z_bottom_a + (iRowIndex * current_box.Height)) + 230;
        }

        if (iRowIndex <= 1)
        {   // 0 || 1

            std::stringstream temp_log_data;
            temp_log_data << u8"type " <<  temp_box.BoxPoseType_
                     << u8" 未更改放置点位置 x:" << temp_box.LoadPose_.x_ << u8" y:" << temp_box.LoadPose_.y_
                     << u8" z:" << temp_box.LoadPose_.z_
                     << u8" pitch_:" << temp_box.LoadPose_.pitch_
                     << u8" roll:" << temp_box.LoadPose_.roll_
                     << u8" yaw:" << temp_box.LoadPose_.yaw_;
            log_.AddLog(__LINE__, __FILE__, temp_log_data.str().c_str(), temp_log_data.str().size(), LLOG::TLogLevel::DEBUG);

            // 由于最底下两层无法 0 俯仰进行上货，所以需要进行补偿
            float fLoadPose[6] = {(float)temp_box.LoadPose_.x_, (float)temp_box.LoadPose_.y_, (float)temp_box.LoadPose_.z_
                                  , (float)(temp_box.LoadPose_.roll_ / (M_PI / 180))
                                  , (float)(temp_box.LoadPose_.pitch_ / (M_PI / 180))
                                  , (float)(temp_box.LoadPose_.yaw_ / (M_PI / 180))};

            float fNewLoadPose[6] = {0};

            float z_off = 0;
            float angle = 0;
//            float x_off = 0;
            if (iRowIndex == 0)
            {
                z_off = 473;
//                angle = 3;
//                x_off = 30;
            }
            else
            {
                z_off = 313;
//                angle = -4;
//                x_off = 15;
            }

            ComputeTcpPose(fLoadPose, 2, z_off, 3, angle, fNewLoadPose);
            temp_box.LoadPose_.SetPose(fNewLoadPose[0], fNewLoadPose[1], fNewLoadPose[2]
                    ,fNewLoadPose[4] * (M_PI / 180), fNewLoadPose[3] * (M_PI / 180), fNewLoadPose[5] * (M_PI / 180));
        }


        qLoadBox_.push(temp_box);

        std::stringstream log_data;
        log_data << u8"type " <<  temp_box.BoxPoseType_
                 << u8"  放置点位置 x:" << temp_box.LoadPose_.x_ << u8" y:" << temp_box.LoadPose_.y_
                 << u8" z:" << temp_box.LoadPose_.z_
                 << u8" pitch_:" << temp_box.LoadPose_.pitch_ / (M_PI / 180)
                 << u8" roll:" << temp_box.LoadPose_.roll_ / (M_PI / 180)
                 << u8" yaw:" << temp_box.LoadPose_.yaw_ / (M_PI / 180);
        log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);

        // 换行
        if (LineFeed)
        {
//            if ((container_z_bottom_a + (iRowIndex * current_box.Height) + current_box.Height + tcp_thickness) > container_z_top_a)
//            if ((container_z_bottom_a + (iRowIndex * current_box.Height) + current_box.Height + 40) > container_z_top_a)
            if (((container_z_bottom_a + ((iRowIndex + 2) * current_box.Height)) > container_z_top_a )
                || iRowIndex >= 14)
            {   // 下一行无法装载了
                break;
            }
            iRowIndex++;
        }
    }


//    if (qLoadBox_.size() > 0)
//    if (false)
    if (qLoadBox_.size() > 0)
    {
        ExecutedBox_ = qLoadBox_.front();
        qLoadBox_.pop();
        bDepthOffset_ = false;  // 用于记录X轴是否已经补偿过
        RobotArmStateData_.ExecuteState_ = ExecuteState::Upload_MoveingtoTarget;    // 移动到装货点
        // 逆解方式 移动到 装货点
        TargetPose_ = ExecutedBox_.LoadPose_;
        float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                        , (float)(TargetPose_.roll_ / (M_PI / 180))
                        , (float)(TargetPose_.pitch_ / (M_PI / 180))
                        , (float)(TargetPose_.yaw_ / (M_PI / 180))};
        RobotArmStateData_.iRobotStatus_ = 2;   // 将状态设置为逆解
        MoveAbs(ZMC_Handle_,6,axis_rt_list,dis, TargetAxisReal_);
    }
    return true;
}

// 真空吸盘的工作函数
void RobotArm::VacuumChuckFunction()
{
    //吸气和吹气开关
    if(iTuboMode_ == 1) //  卸货通用流程
    {
        if (fDpos_13_<100)
        {
            ZAux_Direct_SetOp(ZMC_Handle_,0,0);//关闭
            ZAux_Direct_SetOp(ZMC_Handle_,1,0);
            ZAux_Direct_SetOp(ZMC_Handle_,2,0);
            ZAux_Direct_SetOp(ZMC_Handle_,3,0);
            ZAux_Direct_SetOp(ZMC_Handle_,4,0);
            ZAux_Direct_SetOp(ZMC_Handle_,12,0);

            ZAux_Direct_SetOp(ZMC_Handle_,5,0);
            ZAux_Direct_SetOp(ZMC_Handle_,6,0);
            ZAux_Direct_SetOp(ZMC_Handle_,7,0);
            ZAux_Direct_SetOp(ZMC_Handle_,10,0);
            ZAux_Direct_SetOp(ZMC_Handle_,11,0);
            ZAux_Direct_SetOp(ZMC_Handle_,13,0);
        }
        else if (fDpos_13_ < 230)
        {
            ZAux_Direct_SetOp(ZMC_Handle_,0,0);
            ZAux_Direct_SetOp(ZMC_Handle_,1,0);
            ZAux_Direct_SetOp(ZMC_Handle_,2,0);
            ZAux_Direct_SetOp(ZMC_Handle_,3,0);
            ZAux_Direct_SetOp(ZMC_Handle_,4,0);
            ZAux_Direct_SetOp(ZMC_Handle_,12,0);

            ZAux_Direct_SetOp(ZMC_Handle_,5,1);
            ZAux_Direct_SetOp(ZMC_Handle_,6,1); // 吹气
            ZAux_Direct_SetOp(ZMC_Handle_,7,1);
            ZAux_Direct_SetOp(ZMC_Handle_,10,1);
            ZAux_Direct_SetOp(ZMC_Handle_,11,1);
            ZAux_Direct_SetOp(ZMC_Handle_,13,1);
        }
        else if (fDpos_13_ < 900)
        {
            if ((ExecutedBoxLabel_ == 1)
                || (ExecutedBoxLabel_ == 6)
                || (ExecutedBoxLabel_ == 3))
            {
                ZAux_Direct_SetOp(ZMC_Handle_,0,1); // 吸气
                ZAux_Direct_SetOp(ZMC_Handle_,1,1);
                ZAux_Direct_SetOp(ZMC_Handle_,2,1);
                ZAux_Direct_SetOp(ZMC_Handle_,3,1);
                ZAux_Direct_SetOp(ZMC_Handle_,4,0);
                ZAux_Direct_SetOp(ZMC_Handle_,12,0);

                ZAux_Direct_SetOp(ZMC_Handle_,5,0);
                ZAux_Direct_SetOp(ZMC_Handle_,6,0);
                ZAux_Direct_SetOp(ZMC_Handle_,7,0);
                ZAux_Direct_SetOp(ZMC_Handle_,10,0);
                ZAux_Direct_SetOp(ZMC_Handle_,11,0);
                ZAux_Direct_SetOp(ZMC_Handle_,13,0);
            }
            else if((ExecutedBoxLabel_ == 2)
                || (ExecutedBoxLabel_ == 7)
                || (ExecutedBoxLabel_ == 4))
            {
                ZAux_Direct_SetOp(ZMC_Handle_,0,0); // 吸气
                ZAux_Direct_SetOp(ZMC_Handle_,1,0);
                ZAux_Direct_SetOp(ZMC_Handle_,2,1);
                ZAux_Direct_SetOp(ZMC_Handle_,3,1);
                ZAux_Direct_SetOp(ZMC_Handle_,4,1);
                ZAux_Direct_SetOp(ZMC_Handle_,12,1);

                ZAux_Direct_SetOp(ZMC_Handle_,5,0);
                ZAux_Direct_SetOp(ZMC_Handle_,6,0);
                ZAux_Direct_SetOp(ZMC_Handle_,7,0);
                ZAux_Direct_SetOp(ZMC_Handle_,10,0);
                ZAux_Direct_SetOp(ZMC_Handle_,11,0);
                ZAux_Direct_SetOp(ZMC_Handle_,13,0);
            }
            else if((ExecutedBoxLabel_ == 5)
                     || (ExecutedBoxLabel_ == 9)
                     || (ExecutedBoxLabel_ == 8))
            {
                ZAux_Direct_SetOp(ZMC_Handle_,0,0); // 吸气
                ZAux_Direct_SetOp(ZMC_Handle_,1,1);
                ZAux_Direct_SetOp(ZMC_Handle_,2,1);
                ZAux_Direct_SetOp(ZMC_Handle_,3,1);
                ZAux_Direct_SetOp(ZMC_Handle_,4,1);
                ZAux_Direct_SetOp(ZMC_Handle_,12,0);

                ZAux_Direct_SetOp(ZMC_Handle_,5,0);
                ZAux_Direct_SetOp(ZMC_Handle_,6,0);
                ZAux_Direct_SetOp(ZMC_Handle_,7,0);
                ZAux_Direct_SetOp(ZMC_Handle_,10,0);
                ZAux_Direct_SetOp(ZMC_Handle_,11,0);
                ZAux_Direct_SetOp(ZMC_Handle_,13,0);
            }
            else
            {
                ZAux_Direct_SetOp(ZMC_Handle_,0,1); // 吸气
                ZAux_Direct_SetOp(ZMC_Handle_,1,1);
                ZAux_Direct_SetOp(ZMC_Handle_,2,1);
                ZAux_Direct_SetOp(ZMC_Handle_,3,1);
                ZAux_Direct_SetOp(ZMC_Handle_,4,1);
                ZAux_Direct_SetOp(ZMC_Handle_,12,1);

                ZAux_Direct_SetOp(ZMC_Handle_,5,0);
                ZAux_Direct_SetOp(ZMC_Handle_,6,0);
                ZAux_Direct_SetOp(ZMC_Handle_,7,0);
                ZAux_Direct_SetOp(ZMC_Handle_,10,0);
                ZAux_Direct_SetOp(ZMC_Handle_,11,0);
                ZAux_Direct_SetOp(ZMC_Handle_,13,0);
            }

        }
    }
    else if (iTuboMode_ == 3)   // 模式3 用于纠正两侧的贴边箱子 吸气
    {
        if (fDpos_13_<100)
        {
            ZAux_Direct_SetOp(ZMC_Handle_,0,0);//关闭
            ZAux_Direct_SetOp(ZMC_Handle_,1,0);
            ZAux_Direct_SetOp(ZMC_Handle_,2,0);
            ZAux_Direct_SetOp(ZMC_Handle_,3,0);
            ZAux_Direct_SetOp(ZMC_Handle_,4,0);
            ZAux_Direct_SetOp(ZMC_Handle_,12,0);

            ZAux_Direct_SetOp(ZMC_Handle_,5,0);
            ZAux_Direct_SetOp(ZMC_Handle_,6,0);
            ZAux_Direct_SetOp(ZMC_Handle_,7,0);
            ZAux_Direct_SetOp(ZMC_Handle_,10,0);
            ZAux_Direct_SetOp(ZMC_Handle_,11,0);
            ZAux_Direct_SetOp(ZMC_Handle_,13,0);
        }
        else
        {
            ZAux_Direct_SetOp(ZMC_Handle_,0,1); // 吸气
            ZAux_Direct_SetOp(ZMC_Handle_,1,1);
            ZAux_Direct_SetOp(ZMC_Handle_,2,1);
            ZAux_Direct_SetOp(ZMC_Handle_,3,1);
            ZAux_Direct_SetOp(ZMC_Handle_,4,1);
            ZAux_Direct_SetOp(ZMC_Handle_,12,1);

            ZAux_Direct_SetOp(ZMC_Handle_,5,0);
            ZAux_Direct_SetOp(ZMC_Handle_,6,0);
            ZAux_Direct_SetOp(ZMC_Handle_,7,0);
            ZAux_Direct_SetOp(ZMC_Handle_,10,0);
            ZAux_Direct_SetOp(ZMC_Handle_,11,0);
            ZAux_Direct_SetOp(ZMC_Handle_,13,0);
        }
    }
    else if (iTuboMode_ == 5)   // 模式5
    {
        if (fDpos_13_ < -700 || RobotArmStateData_.umapIoInput_[9] > 0)
        {
            ZAux_Direct_SetOp(ZMC_Handle_,0,0);//关闭
            ZAux_Direct_SetOp(ZMC_Handle_,1,0);
            ZAux_Direct_SetOp(ZMC_Handle_,2,0);
            ZAux_Direct_SetOp(ZMC_Handle_,3,0);
            ZAux_Direct_SetOp(ZMC_Handle_,4,0);
            ZAux_Direct_SetOp(ZMC_Handle_,12,0);

            ZAux_Direct_SetOp(ZMC_Handle_,5,0);
            ZAux_Direct_SetOp(ZMC_Handle_,6,0);
            ZAux_Direct_SetOp(ZMC_Handle_,7,0);
            ZAux_Direct_SetOp(ZMC_Handle_,10,0);
            ZAux_Direct_SetOp(ZMC_Handle_,11,0);
            ZAux_Direct_SetOp(ZMC_Handle_,13,0);
        }
        else if (fDpos_13_ < -360)
        {
            ZAux_Direct_SetOp(ZMC_Handle_,0,0);//吹气
            ZAux_Direct_SetOp(ZMC_Handle_,1,0);
            ZAux_Direct_SetOp(ZMC_Handle_,2,0);
            ZAux_Direct_SetOp(ZMC_Handle_,3,0);
            ZAux_Direct_SetOp(ZMC_Handle_,4,0);
            ZAux_Direct_SetOp(ZMC_Handle_,12,0);

            ZAux_Direct_SetOp(ZMC_Handle_,5,1);
            ZAux_Direct_SetOp(ZMC_Handle_,6,1);
            ZAux_Direct_SetOp(ZMC_Handle_,7,1);
            ZAux_Direct_SetOp(ZMC_Handle_,10,1);
            ZAux_Direct_SetOp(ZMC_Handle_,11,1);
            ZAux_Direct_SetOp(ZMC_Handle_,13,1);
        }
        else
        {
            ZAux_Direct_SetOp(ZMC_Handle_,0,0);
            ZAux_Direct_SetOp(ZMC_Handle_,1,1);
            ZAux_Direct_SetOp(ZMC_Handle_,2,1);
            ZAux_Direct_SetOp(ZMC_Handle_,3,1);
            ZAux_Direct_SetOp(ZMC_Handle_,4,1);
            ZAux_Direct_SetOp(ZMC_Handle_,12,0);

            ZAux_Direct_SetOp(ZMC_Handle_,5,0);
            ZAux_Direct_SetOp(ZMC_Handle_,6,0);
            ZAux_Direct_SetOp(ZMC_Handle_,7,0);
            ZAux_Direct_SetOp(ZMC_Handle_,10,0);
            ZAux_Direct_SetOp(ZMC_Handle_,11,0);
            ZAux_Direct_SetOp(ZMC_Handle_,13,0);
        }
    }
    else if (iTuboMode_ == 4)   // 模式4 用于纠正两侧的贴边箱子 吹气
    {
        if (fDpos_13_ < 100)
        {
            ZAux_Direct_SetOp(ZMC_Handle_,0,0);//关闭
            ZAux_Direct_SetOp(ZMC_Handle_,1,0);
            ZAux_Direct_SetOp(ZMC_Handle_,2,0);
            ZAux_Direct_SetOp(ZMC_Handle_,3,0);
            ZAux_Direct_SetOp(ZMC_Handle_,4,0);
            ZAux_Direct_SetOp(ZMC_Handle_,5,0);
            ZAux_Direct_SetOp(ZMC_Handle_,6,0);
            ZAux_Direct_SetOp(ZMC_Handle_,7,0);
            ZAux_Direct_SetOp(ZMC_Handle_,10,0);
            ZAux_Direct_SetOp(ZMC_Handle_,11,0);
            ZAux_Direct_SetOp(ZMC_Handle_,12,0);
            ZAux_Direct_SetOp(ZMC_Handle_,13,0);
        }
        else
        {
            ZAux_Direct_SetOp(ZMC_Handle_, 0, 0);
            ZAux_Direct_SetOp(ZMC_Handle_, 1, 0);
            ZAux_Direct_SetOp(ZMC_Handle_, 2, 0);
            ZAux_Direct_SetOp(ZMC_Handle_, 3, 0);
            ZAux_Direct_SetOp(ZMC_Handle_, 4, 0);
            ZAux_Direct_SetOp(ZMC_Handle_, 12, 0);

            ZAux_Direct_SetOp(ZMC_Handle_, 5, 1);
            ZAux_Direct_SetOp(ZMC_Handle_, 6, 1);// 吹气
            ZAux_Direct_SetOp(ZMC_Handle_, 7, 1);
            ZAux_Direct_SetOp(ZMC_Handle_, 10, 1);
            ZAux_Direct_SetOp(ZMC_Handle_, 11, 1);
            ZAux_Direct_SetOp(ZMC_Handle_, 13, 1);
        }
    }

}

void RobotArm::ToPhotosPosture(int pose_id)
{
    if (RobotArmStateData_.RobotArmState_ == RobotArmState::Normal)
    {
        //llSendTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        if(ZMC_Handle_ == nullptr)
        {
            qDebug() << u8"警告  " << "未链接到控制器";
        }

        if (pose_id == 1)
        {
//            TargetPose_.SetPose(5170,0,1850,0,0,0); //上拍照点
            TargetPose_.SetPose(5400, 0, 2100, -7 * (M_PI / 180), 0, 0); //上拍照点
        }
        else if (pose_id == 2)
        {
//            TargetPose_.SetPose(5170,0,1500,0,0,0); //下拍照点
            TargetPose_.SetPose(5400, 0, 1300, 0, 0, 0); //下拍照点
        }
        else
        {
            return;
        }

        float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                                , (float)(TargetPose_.roll_ / (M_PI / 180))
                                , (float)(TargetPose_.pitch_ / (M_PI / 180))
                                , (float)(TargetPose_.yaw_ / (M_PI / 180))};

        RobotArmStateData_.iRobotStatus_ = 2;
        MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
        RobotArmStateData_.ExecuteState_ = ExecuteState::SinglePointMove;
        pStateCallback_(RobotArmStateData_);
    }
}


//  移动到测距姿势
void RobotArm::ToRangingPosture(int pose_id)
{
    if (pose_id == 0)
    {
        TargetPose_.SetPose(5400, 0, 900, 0, 0, 0);
    }
    else if (pose_id == 1)
    {
        TargetPose_.SetPose(5400, 250, 550, 0, 0, 0);
    }
    else if (pose_id == 2)
    {
        TargetPose_.SetPose(5400, -250, 600, 0, 0, 0);
    }
    else
    {
       TargetPose_.SetPose(5400, 0, 950, 0, 0, 0);
    }

    float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                            , (float)(TargetPose_.roll_ / (M_PI / 180))
                            , (float)(TargetPose_.pitch_ / (M_PI / 180))
                            , (float)(TargetPose_.yaw_ / (M_PI / 180))};

    RobotArmStateData_.iRobotStatus_ = 2;
    MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
    RobotArmStateData_.ExecuteState_ = ExecuteState::SinglePointMove;
    pStateCallback_(RobotArmStateData_);
}


//  移动到折叠姿势
void RobotArm::ToFoldingPosture()
{
    TargetPose_.SetPose(5400, 0, 900, 0, 0, 0);
    float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                            , (float)(TargetPose_.roll_ / (M_PI / 180))
                            , (float)(TargetPose_.pitch_ / (M_PI / 180))
                            , (float)(TargetPose_.yaw_ / (M_PI / 180))};

    RobotArmStateData_.iRobotStatus_ = 2;
    MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
    RobotArmStateData_.ExecuteState_ = ExecuteState::SinglePointMove;
    pStateCallback_(RobotArmStateData_);
}


void RobotArm::MoveArm(Common::Pose target_pose)
{
    TargetPose_ = target_pose;
    float dis[6] = {(float)TargetPose_.x_, (float)TargetPose_.y_, (float)TargetPose_.z_
                            , (float)(TargetPose_.roll_ / (M_PI / 180))
                            , (float)(TargetPose_.pitch_ / (M_PI / 180))
                            , (float)(TargetPose_.yaw_ / (M_PI / 180))};

    RobotArmStateData_.iRobotStatus_ = 2;
    MoveAbs(ZMC_Handle_, 6, axis_rt_list, dis, TargetAxisReal_);
    RobotArmStateData_.ExecuteState_ = ExecuteState::SinglePointMove;
    pStateCallback_(RobotArmStateData_);
}

void RobotArm::CameraControl(int mode_id)
{
    if (mode_id == 0)   // 关闭
    {
        ZAux_Direct_SetOp(ZMC_Handle_, 8, 0);
        ZAux_Direct_SetOp(ZMC_Handle_, 9, 0);
    }
    else    // 开启
    {
        ZAux_Direct_SetOp(ZMC_Handle_, 8, 1);
        ZAux_Direct_SetOp(ZMC_Handle_, 9, 1);
    }
}

void RobotArm::RunSingleAxis(const RobotArmControl& control_data)
{
    if((ZMC_Handle_ != nullptr && RobotArmStateData_.RobotArmState_ == RobotArmState::Normal)
       && (RobotArmStateData_.ExecuteState_ == ExecuteState::Idle))
    {
        int isidle = 0;
        ZAux_Direct_GetIfIdle(ZMC_Handle_, control_data.iAxisGroupIndex_, &isidle);
        if (-1 != isidle)
        {
            qDebug()<<"not stop";
            return;
        }
        ZAux_Direct_SetUnits(ZMC_Handle_, control_data.iAxisGroupIndex_, control_data.dUnits_);
        ZAux_Direct_SetSpeed(ZMC_Handle_, control_data.iAxisGroupIndex_, control_data.dSpeed_);
        ZAux_Direct_SetAccel(ZMC_Handle_, control_data.iAxisGroupIndex_, control_data.dAccel_);
        ZAux_Direct_SetDecel(ZMC_Handle_, control_data.iAxisGroupIndex_, control_data.dDecel_);
        ZAux_Direct_SetSramp(ZMC_Handle_, control_data.iAxisGroupIndex_, control_data.dSramp_);

        if(1 == control_data.iRunModeGroupIndex_)//inch
        {
           ZAux_Direct_Singl_Move(ZMC_Handle_, control_data.iAxisGroupIndex_,
                                  control_data.bMotorDirection_ ? -control_data.dMoveDistance_ :
                                                                  control_data.dMoveDistance_);
        }
        else if(0 == control_data.iRunModeGroupIndex_)//continue
        {
            ZAux_Direct_Singl_Vmove(ZMC_Handle_, control_data.iAxisGroupIndex_,
                                    control_data.bMotorDirection_ ? -1 : 1);
        }
    }
}

void RobotArm::StopSingleAxis(const RobotArmControl& control_data)
{
    if(ZMC_Handle_ != nullptr
       && RobotArmStateData_.RobotArmState_ == RobotArmState::Normal)
    {
//        int axislist[8]={0,1,2,3,4,14,12,13};
        int isidle = 0;
        ZAux_Direct_GetIfIdle(ZMC_Handle_, control_data.iAxisGroupIndex_, &isidle);
        if(isidle != 0)
        {
            return;
        }
        ZAux_Direct_Singl_Cancel(ZMC_Handle_, control_data.iAxisGroupIndex_,2);
//        ZAux_Direct_CancelAxisList(ZMC_Handle_, 7, axislist, 2);
    }
}

void RobotArm::SetLimitPoint_(int type, double dValue)
{
    if (type == 0)
    {   // 左
        RobotArmStateData_.dLeftLimitPoint_ = dValue;
//        RobotArmStateData_.dLeftLimitPoint_ -= (555 / 2) + 30; // 555 是小臂的宽，30为安全距离
    }
    else
    {
        RobotArmStateData_.dRightLimitPoint_ = dValue;
//        RobotArmStateData_.dRightLimitPoint_ += (555 / 2) + 30; // 555 是小臂的宽，30为安全距离
    }
}

void RobotArm::SingleAxisReset(const RobotArmControl& control_data)
{
    if(ZMC_Handle_ != nullptr
       && RobotArmStateData_.RobotArmState_ == RobotArmState::Normal)
    {
        int isidle = 0;
        ZAux_Direct_GetIfIdle(ZMC_Handle_, control_data.iAxisGroupIndex_, &isidle);
        if(!isidle) return;
        ZAux_Direct_SetDpos(ZMC_Handle_, control_data.iAxisGroupIndex_, 0);
    }
}

void RobotArm::SingleAxisResetCoder(const RobotArmControl& control_data)
{
    if(ZMC_Handle_ != nullptr
       && RobotArmStateData_.RobotArmState_ == RobotArmState::Normal)
    {
        int isidle = 0;
        ZAux_Direct_GetIfIdle(ZMC_Handle_, control_data.iAxisGroupIndex_, &isidle);
        if(!isidle) return;
        ZAux_Direct_SetMpos(ZMC_Handle_, control_data.iAxisGroupIndex_, 0);
    }
}

// 停止全部轴
void RobotArm::StopAllAxis()
{
    ZAux_Direct_Rapidstop(ZMC_Handle_,2);
    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 0, 0);
    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 1, 0);
    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 2, 0);
    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 3, 0);
    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 4, 0);
    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 12, 0);
    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 13, 0);
    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 14, 0);
    ZAux_Direct_SetAxisEnable(ZMC_Handle_, 15, 0);

    RobotArmStateData_.ExecuteState_ = ExecuteState::Stop;
}

// 暂停任务
void RobotArm::Pause()
{
    // 记录当前状态
    PauseBackup();
    StopAllAxis();  // 停止所有轴
    bRobotArmPause_ = true;
    RobotArmStateData_.ExecuteState_ = ExecuteState::Pause; // 更换成 暂停状态
    std::string strLog("触发暂停");
    log_.AddLog(__LINE__, __FILE__, strLog.c_str(), strLog.size(), LLOG::TLogLevel::WARNING);
}

// 暂停备份
void RobotArm::PauseBackup()
{
    PauseBackupData_.ExecuteState_ = RobotArmStateData_.ExecuteState_;
    PauseBackupData_.TargetPose_ = TargetPose_;
    PauseBackupData_.ExecutedBox_ = ExecutedBox_;

    std::stringstream log_data;
    log_data << "备份  " << "ExecuteState_:" << RobotArmStateData_.ExecuteState_
             << "  TargetPose_:" << TargetPose_.x_ << "," << TargetPose_.y_ << "," << TargetPose_.z_
             << "  ExecutedBox_:" << ExecutedBox_.LoadPose_.x_<< "," << ExecutedBox_.LoadPose_.y_ << "," << ExecutedBox_.LoadPose_.z_;
    log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::WARNING);
}

// 恢复运行
void RobotArm::ResumeToRun()
{
    if (RobotArmStateData_.ExecuteState_ == ExecuteState::Pause)
    {
        // 需要完成 上电和建立模型
        RobotArmStateData_.ExecuteState_ = ExecuteState::Resume;
        bRobotArmPause_ = false;
        bRobotArmResume_ = true;
        std::string strLog("触发恢复");
        log_.AddLog(__LINE__, __FILE__, strLog.c_str(), strLog.size(), LLOG::TLogLevel::WARNING);
    }
}

// 复位
void RobotArm::Reset()
{
    // 清除所有的内存
    while (!qLoadBox_.empty())
    {
        qLoadBox_.pop();
    }

    while (!qUnloadBox_.empty())
    {
        qUnloadBox_.pop();
    }

    bRepeatFunction_ = false;
    llGraspWaitTime_ = 0;
    bTryGrasp_ = false;
    ZMC_Handle_ = nullptr;
    bRobotArmResume_ = false;
    bRobotArmPause_ = false;
    RobotArmStateData_.ExecuteState_ = ExecuteState::Idle;
    pStateCallback_(RobotArmStateData_);
}

// 返回短距离测距 的距离 返回毫米单位
//double RobotArm::GetPointLaserDistance()
//{
//    return RobotArmStateData_.dLaserDistance_;
//}
// 返回 短距离测距 的距离 返回毫米单位
//double RobotArm::GetPointLaserShortDistance()
//{
//    return RobotArmStateData_.dLaserShortDist_;
//}

// 返回 长距离测距 的距离 返回毫米单位
double RobotArm::GetPointLaserLongDistance()
{
    return RobotArmStateData_.dLaserLongDist_;
}

// 返回 左侧测距 的距离 返回毫米单位
double RobotArm::GetPointLaserLeftDistance()
{
    return RobotArmStateData_.dLaserLeftDist_;
}

double RobotArm::GetPointLaserDownDistance()
{
    return RobotArmStateData_.dLaserDownDist_;
}

void RobotArm::TcpHoming()
{
    if(nullptr == ZMC_Handle_)
    {
        qDebug()<<"not link control!!";
        return;
    }
    int status = 0;
    ZAux_Direct_GetIfIdle(ZMC_Handle_, 13, &status);
    if (status == 0) //已经在运动中
    {
        return;
    }
    //设定对应轴的原点输入口信号
    BigarmSuctionCupBracket_.SingleDatum(ZMC_Handle_, 3);
    BigarmSuctionCupBracket_.SetDpos(ZMC_Handle_, 0);
    BigarmSuctionCupBracket_.SetMpos(ZMC_Handle_, 0);
}

// 吸盘架推出
void RobotArm::TcpPush()
{
    if(0 == ZMC_Handle_)
    {
        qDebug()<<"not link control!!";
        return;
    }
    int iRemainBuffer;
    BigarmSuctionCupBracket_.GetRemainBuffer(ZMC_Handle_, &iRemainBuffer); // 获取13轴剩余的缓存空间
    if(iRemainBuffer > 0)
    {
        BigarmSuctionCupBracket_.SingleMoveSp(ZMC_Handle_, 630); // 吸盘架全行程 dpos 是 640
    }
}

void RobotArm::GetTcpData()
{
    qDebug() << "GetTcpData";
    float dis_real_list[6] = {0};
    dis_real_list[0] = RobotArmStateData_.fL_1_; //五个轴的关节角度
    dis_real_list[1] = RobotArmStateData_.fL_2_;
    dis_real_list[2] = RobotArmStateData_.fL_3_;
    dis_real_list[3] = RobotArmStateData_.fL_4_;
    dis_real_list[4] = RobotArmStateData_.fL_5_;

    ZAux_Direct_SetTable(ZMC_Handle_, 1100, 6, dis_real_list);
    ZAux_GetFrameTrans2(ZMC_Handle_, axis_real_list, 6, 1100, 1150, 1);
    float dis_rt_list[6] = {0};
    ZAux_Direct_GetTable(ZMC_Handle_, 1150, 6, dis_rt_list);

    RobotArmStateData_.fTcp_x_ = dis_rt_list[0]; //tcp位姿坐标
    RobotArmStateData_.fTcp_y_ = dis_rt_list[1];
    RobotArmStateData_.fTcp_z_ = dis_rt_list[2];
    RobotArmStateData_.fTcp_rx_ = dis_rt_list[3];
    RobotArmStateData_.fTcp_ry_ = dis_rt_list[4];
    RobotArmStateData_.fTcp_rz_ = dis_rt_list[5];
}

void RobotArm::SetLeftEdgeReturnAngle(float fValue)
{
    RobotArmStateData_.fLeftEdgeReturnAngle_ = fValue;
}

void RobotArm::SetLeftEdgeReturnDis(float fValue)
{
    RobotArmStateData_.fLeftEdgeReturnDis_ = fValue;
}

void RobotArm::SetLeftEdgeReturnXDis(float fValue)
{
    RobotArmStateData_.fLeftEdgeReturnXDis_ = fValue;
}

void RobotArm::SetRightEdgeReturnAngle(float fValue)
{
    RobotArmStateData_.fRightEdgeReturnAngle_ = fValue;
}

void RobotArm::SetRightEdgeReturnDis(float fValue)
{
    RobotArmStateData_.fRightEdgeReturnDis_ = fValue;
}

void RobotArm::SetRightEdgeReturnXDis(float fValue)
{
    RobotArmStateData_.fRightEdgeReturnXDis_ = fValue;
}

void RobotArm::tcp_single_run()//末端抓取
{
    if(nullptr == ZMC_Handle_)
    {
        qDebug()<<"not link control!!";
        return;
    }
    int iRemainBuffer;
    BigarmSuctionCupBracket_.GetRemainBuffer(ZMC_Handle_, &iRemainBuffer);
    if(iRemainBuffer > 0)
    {
        qDebug() << "tcp_single_run" <<  __LINE__;
        BigarmSuctionCupBracket_.SingleMoveSp(ZMC_Handle_, 625); // 吸盘架全行程 dpos 是 640
        BigarmSuctionCupBracket_.MoveDelay(ZMC_Handle_, 500);
        //**********************************************
        BigarmSuctionCupBracket_.SingleMoveSp(ZMC_Handle_, -605); // 吸盘架全行程 dpos 是 640

        BigarmSuctionCupBracket_.SingleDatum(ZMC_Handle_, 3);
        BigarmSuctionCupBracket_.SetDpos(ZMC_Handle_, 0);
        BigarmSuctionCupBracket_.SetMpos(ZMC_Handle_, 0);
    }
}

void RobotArm::tcp_single_run(float start_pos)//末端抓取
{
    if(nullptr == ZMC_Handle_)
    {
        qDebug()<<"not link control!!";
        return;
    }
    qDebug() << "start_dpos  :  " << start_pos;
    int iRemainBuffer;
    BigarmSuctionCupBracket_.GetRemainBuffer(ZMC_Handle_, &iRemainBuffer);
    if(iRemainBuffer > 0) {
        BigarmSuctionCupBracket_.SingleMoveSp(ZMC_Handle_, 85);
        BigarmSuctionCupBracket_.MoveDelay(ZMC_Handle_, 500);
        //**********************************************
        BigarmSuctionCupBracket_.SingleMoveSp(ZMC_Handle_, -605); // 吸盘架全行程 dpos 是 640

        BigarmSuctionCupBracket_.SingleDatum(ZMC_Handle_, 3);
        BigarmSuctionCupBracket_.SetDpos(ZMC_Handle_, 0);
        BigarmSuctionCupBracket_.SetMpos(ZMC_Handle_, 0);
    }
}

bool RobotArm::isArriveThisPlace(float array[6])
{
   if (       (RobotArmStateData_.fL_1_ >= array[0]-0.1 && RobotArmStateData_.fL_1_ <= array[0]+0.1)
           && (RobotArmStateData_.fL_2_ >= array[1]-0.1 && RobotArmStateData_.fL_2_ <= array[1]+0.1)
           && (RobotArmStateData_.fL_3_ >= array[2]-0.1 && RobotArmStateData_.fL_3_ <= array[2]+0.1)
           && (RobotArmStateData_.fL_4_ >= array[3]-0.1 && RobotArmStateData_.fL_4_ <= array[3]+0.1)
           && (RobotArmStateData_.fL_5_ >= array[4]-0.1 && RobotArmStateData_.fL_5_ <= array[4]+0.1)
           && (RobotArmStateData_.fL_6_ >= array[5]-0.1 && RobotArmStateData_.fL_6_ <= array[5]+0.1))
   {
       return true;
   }else
   {
       return false;
   }
}


// direct to inverse kinematic
//void RobotArm::DirectToInverseFrameTans(float* pfDirectPose, float fAngle, float* pfInversePose)
//{

//    float TatgetPose[6] ={0};
//    float InversePose[6] ={0};

//    TatgetPose[0] = pfDirectPose[0];
//    TatgetPose[1] = pfDirectPose[1];
//    TatgetPose[2] = pfDirectPose[2];
//    TatgetPose[3] = pfDirectPose[3];
//    TatgetPose[5] = pfDirectPose[5];
//    if(-15 < fAngle && fAngle < 15)
//    {
//        TatgetPose[4] = fAngle;
//    }
//    else
//    {
//        qDebug()<<"偏航角度过大，超出限位";
//        return ;
//    }
//    // TatgetPose->setpos(4, fAngle);

//    auto ret = ZAux_Direct_SetTable(ZMC_Handle_, 660, 6, TatgetPose);
//    if(ret)
//    {
//        // QMessageBox::warning(this, "警告",QString("转换出错，错误码为：%1").arg(ret));
//        qDebug()<<"设置正解寄存器失败";
//        return ;
//    }

//    ZAux_Direct_Connreframe(ZMC_Handle_, 6, axis_rt_list, 121, 10, 6, axis_real_list);//建立正解模式
//    ret = ZAux_GetFrameTrans2(ZMC_Handle_, axis_real_list, 6, 660, 670, 3);
//    if(ret)
//    {
//        // QMessageBox::warning(this, "警告",QString("转换出错，错误码为：%1").arg(ret));
//        qDebug()<<"坐标转换失败";
//        return ;
//    }
//    ret = ZAux_Direct_GetTable(ZMC_Handle_, 670, 6, InversePose);
//    if(ret)
//    {
//        // QMessageBox::warning(this, "警告",QString("转换出错，错误码为：%1").arg(ret));
//        qDebug()<<"获取逆解寄存器失败";
//        return ;
//    }

//    pfInversePose[0] = InversePose[0];
//    pfInversePose[1] = InversePose[1];
//    pfInversePose[2] = InversePose[2];
//    pfInversePose[3] = InversePose[3];
//    pfInversePose[4] = InversePose[4];
//    pfInversePose[5] = InversePose[5];

//    return;
//}


bool RobotArm::MoveAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist
                       , float *pfDisancelist, float(&pdTargetAxisReal)[6])
{
    float out_list[6] ={0};
    ZAux_Direct_SetTable(ZMC_Handle_, 800, 6, pfDisancelist);
    qDebug() << "MoveAbs 输入的逆解 " << pfDisancelist[0] << "," << pfDisancelist[1] << ","
             << pfDisancelist[2] << "," << pfDisancelist[3] << "," << pfDisancelist[4] << pfDisancelist[5];

    int ret = ZAux_GetFrameTrans2(ZMC_Handle_, axis_rt_list, 6, 800, 850, 0);
    qDebug() << "ZAux_GetFrameTrans2 return value=" << ret;
    if(ret)
    {
        qDebug()<<"建立逆解出错，重新建立逆解";
        return false;
    }
    ZAux_Direct_GetTable(ZMC_Handle_, 850, 6, out_list);

    qDebug() << "MoveAbs 正解 " << out_list[0] << "," << out_list[1] << ","
             << out_list[2] << "," << out_list[3] << "," << out_list[4] << out_list[5];
    std::copy(std::begin(out_list), std::end(out_list), std::begin(pdTargetAxisReal));
    // 15,  38, 35.5, 25,  15, 15
    // -15,-35,-61.5,-25,-15,-15
    if(out_list[0] > -15 && out_list[0] < 15
        && out_list[1] > -35 && out_list[1] < 38
        && out_list[2] > -61.5 && out_list[2] < 35.5
        && out_list[3] > -25 && out_list[3] < 25
        && out_list[4] > -15 && out_list[4] < 15)
    {
        ZAux_Direct_MoveAbs(ZMC_Handle_, imaxaxises, piAxislist, pfDisancelist);
    }
    else
    {
        //QMessageBox::warning(this, "警告", QString("超出机械限定范围"));
        std::stringstream log_data1;
        log_data1 << u8" 超出机械限定范围 " ;
        log_.AddLog(__LINE__, __FILE__, log_data1.str().c_str(), log_data1.str().size(), LLOG::TLogLevel::ERROR);
        qDebug()<<"超出机械限定范围";
        qDebug()<<"Dpos(0)="<<out_list[0];
        qDebug()<<"Dpos(1)="<<out_list[1];
        qDebug()<<"Dpos(2)="<<out_list[2];
        qDebug()<<"Dpos(3)="<<out_list[3];
        qDebug()<<"Dpos(4)="<<out_list[4];
    }
    return true;
}

bool RobotArm::IsEffectivePose(Common::Pose &pose)
{
    float dis[6] = {(float)pose.x_, (float)pose.y_, (float)pose.z_
                            , (float)(pose.roll_ / (M_PI / 180))
                            , (float)(pose.pitch_ / (M_PI / 180))
                            , (float)(pose.yaw_ / (M_PI / 180))};
    float out_list[6] ={0};
    ZAux_Direct_SetTable(ZMC_Handle_, 1300, 6, dis);
    int ret = ZAux_GetFrameTrans2(ZMC_Handle_, axis_rt_list, 6, 1300, 1350, 0);
    if(ret)
    {
        qDebug()<<"建立逆解出错，重新建立逆解";
        return false;
    }
    ZAux_Direct_GetTable(ZMC_Handle_, 1350, 6, out_list);
    // 15,  38, 35.5, 25,  15, 15
    // -15,-35,-61.5,-25,-15,-15
    if( !(out_list[0] > -15 && out_list[0] < 15
        && out_list[1] > -35 && out_list[1] < 38
        && out_list[2] > -61.5 && out_list[2] < 35.5
        && out_list[3] > -25 && out_list[3] < 25
        && out_list[4] > -15 && out_list[4] < 15) )
    {
        return false;
    }
    return true;
}

bool RobotArm::SetAxisSpeed(int* axis_list, int list_size, float speed, float accel, float decel)
{
    if (!ZMC_Handle_)
    {
        return false;
    }
    for (int i = 0; i < list_size; i++)
    {
        ZAux_Direct_SetSpeed(ZMC_Handle_, axis_list[i], speed);
        ZAux_Direct_SetAccel(ZMC_Handle_, axis_list[i], accel);
        ZAux_Direct_SetDecel(ZMC_Handle_, axis_list[i], decel);
    }
    return true;
}

// 判断前臂是不是存在货物
bool RobotArm::ForearmIsExistBox()
{
    /*  ------------|>]------------
     *  |10______12 |>]13         |
     *              |>]======== 9 |
     *  |----11-----|>]    14  15 |
     *  ------------|>]---------- -
    */
    // 判断推杆是不是推出， 推杆会影响前两个光电传感器
    if ((RobotArmStateData_.umapIoInput_[9] > 0)    // 9：推杆的限位
       && ((RobotArmStateData_.umapIoInput_[10] > 0)
       || (RobotArmStateData_.umapIoInput_[11] > 0)
       || (RobotArmStateData_.umapIoInput_[12] > 0)))
    {
        return true;
    }

    if ((RobotArmStateData_.umapIoInput_[13] > 0)
       ||(RobotArmStateData_.umapIoInput_[14] > 0)
       ||(RobotArmStateData_.umapIoInput_[15] > 0))
    {
        return true;
    }
    return false;
}

bool RobotArm::BigArmBeltRun(float fSpeed, int iDir)
{
    if (!ZMC_Handle_)
    {
        return false;
    }
    fSpeed = 0.0;
    ZAux_Direct_SetUnits(ZMC_Handle_, 14, 552730);
    ZAux_Direct_SetAccel(ZMC_Handle_, 14, 2000);
    ZAux_Direct_SetDecel(ZMC_Handle_, 14, 10000);
    ZAux_Direct_SetSpeed(ZMC_Handle_, 14, fSpeed);//轴14 大臂传送带运动 装货
    ZAux_Direct_Single_Vmove(ZMC_Handle_, 14, iDir);
    return true;
}

bool RobotArm::BigArmBeltStop()
{
     ZAux_Direct_Single_Cancel(ZMC_Handle_, 14, 2);
     return true;
}

// direct to inverse kinematic
void RobotArm::DirectToInverseFrameTans(float* direct_pose, float* inverse_pose)
{

    auto ret = ZAux_Direct_SetTable(ZMC_Handle_, 900, 6, direct_pose);
    if(ret)
    {
        qDebug()<<"设置正解寄存器失败";
        return ;
    }
//    ZAux_Direct_Connreframe(ZMC_Handle_, 6, axis_rt_list, 121, 10, 6, axis_real_list);//建立正解模式
    ret = ZAux_GetFrameTrans2(ZMC_Handle_, axis_real_list, 6, 900, 950, 3); // 3
    if(ret)
    {
        qDebug() <<  __LINE__  <<" 坐标转换失败  " << ret;
        return ;
    }
    ret = ZAux_Direct_GetTable(ZMC_Handle_, 950, 6, inverse_pose);
    if(ret)
    {
        qDebug()<<"获取逆解寄存器失败";
        return ;
    }

    return;
}

// inverse to direct kinematic
void RobotArm::InverseToDirectFrameTans(float* inverse_pose, float* direct_pose)
{

    auto ret = ZAux_Direct_SetTable(ZMC_Handle_, 1000, 6, inverse_pose);
    if(ret)
    {
        qDebug()<<"设置逆解寄存器失败";
        return ;
    }

//    ZAux_Direct_Connframe(ZMC_Handle_, 6, axis_real_list, 121, 10, 6, axis_rt_list);//建立逆解模式
    ret = ZAux_GetFrameTrans2(ZMC_Handle_, axis_rt_list, 6, 1000, 1050, 0); // 2
    if(ret)
    {
        qDebug()<< __LINE__ << " 坐标转换失败  "  << ret;
        return ;
    }

    ret = ZAux_Direct_GetTable(ZMC_Handle_, 1050, 6, direct_pose);
    if(ret)
    {
        qDebug()<<"获取正解寄存器失败";
        return ;
    }

    return;
}

void RobotArm::ComputeTcpPose(float* temp_pose, int position_num, float distance,
                               int rotation_num, float angle, float* target_pose)
{
    float fDirectPose[6] ={0};
    temp_pose[position_num] = temp_pose[position_num] + distance;

    InverseToDirectFrameTans(temp_pose, fDirectPose);
    fDirectPose[rotation_num] = angle;
    DirectToInverseFrameTans(fDirectPose, target_pose);
}

bool RobotArm::GetPhotoSensorIsTriggered(int ID)
{
    bool state = false;
    try
    {
        state = RobotArmStateData_.umapIoInput_.at(ID) > 0 ? true : false;
    }
    catch(const std::out_of_range &e)
    {
        std::stringstream log_data;
        log_data << u8"没有 ID=" << ID << " 的光电设备" ;
        log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::ERROR);
    }
    return state;
}

}


