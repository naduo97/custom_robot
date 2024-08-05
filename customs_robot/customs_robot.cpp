#include "customs_robot.h"
#include <iostream>
#include <QDebug>
#include <termios.h>
// Xiaomi Pro 的改动
// Airs_Linux 的改动
namespace CustomsRobot{
Robot::Robot()
    : pRobotStateData_(nullptr)
    , log_(LLOG::TLogLevel::DEBUG, "CustomsRobot", "./log", 200)
{
    iWaitPointID_ = 14;
    llWaitTime_ = ULLONG_MAX;
    iAdjustmentPointID_ = -1;
    iIdentificationRegion_ = 1;
    pTaskManagement_ = nullptr;
    pRobotChassis_ = nullptr;
    pRobotArm_ = nullptr;
    pVisualIdentity_ = nullptr;

    bNavArrive_ = false;
    bArmArrive_ = false;
    bVisualFulfil_ = false;
    bCompleteMount_ = false;
    bCompleteUnMount_ = false;

    //装货流程 by JF Gan 
    bManualStartLoad_ = false;
    bManualStopLoad_ = false;
    bManualStopMission_ = false;

    iConfirmVisualResultsMode_ = 0;

//    JrtBb2x_.ContinuousAcquisitionSlowMeasurement();
    Init();
    //processed
}


Robot::Robot(UPDATE_CALLBACK updata_callback,\
             ERROR_CALLBACK error_callback)
    : pRobotStateData_(nullptr)
    , log_(LLOG::TLogLevel::DEBUG, "CustomsRobot", "./log", 200)
{
    iWaitPointID_ = 14;
    llWaitTime_ = ULLONG_MAX;
    iAdjustmentPointID_ = -1;
    iIdentificationRegion_ = 1;

    pUpdateCallback = updata_callback;
    pErrorCallback_ = error_callback;

    pTaskManagement_ = nullptr;
    pRobotChassis_ = nullptr;
    pRobotArm_ = nullptr;
    pVisualIdentity_ = nullptr;

    bNavArrive_ = false;
    bArmArrive_ = false;
    bVisualFulfil_ = false;
    bCompleteMount_ = false;
    bCompleteUnMount_ = false;

    //装货流程 by JF Gan 
    bManualStartLoad_ = false;         
    bManualStopLoad_ = false;           
    bManualStopMission_ = false;        

    bRobotResume = false;

//    JrtBb2x_.ContinuousAcquisitionSlowMeasurement();
    Init();
}


Robot::~Robot()
{
    Stop();
    thWork_.join();
    delete pRobotStateData_;
    pRobotStateData_ = NULL;
    if (pTaskManagement_ != nullptr)
    {
        delete pTaskManagement_;
        pTaskManagement_ = NULL;
    }
    if (pRobotChassis_ != nullptr)
    {
        delete pRobotChassis_;
        pRobotChassis_ = NULL;
    }
    if (pRobotArm_ != nullptr)
    {
        delete pRobotArm_;
        pRobotArm_ = NULL;
    }
    if (pVisualIdentity_ != nullptr)
    {
        delete pVisualIdentity_;
        pVisualIdentity_ = NULL;
    }
    qDebug() << "~CustomsRobot";
}


bool Robot::Init()
{
    pRobotStateData_ = new RoboStateDate();
    robot_state_ = RobotState::Uninitialized;
    bIsStop_ = false;

    umapWorkFun.insert(std::make_pair(RobotState::Uninitialized, std::bind(&Robot::Uninitialized, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Ready, std::bind(&Robot::Ready, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Pause, std::bind(&Robot::Pause, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Resume, std::bind(&Robot::Resume, this)));
    // 卸货 流程
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ToMissionPoint, std::bind(&Robot::Unload_ToMissionPoint, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ArmToPhotosPosture, std::bind(&Robot::Unload_ArmToPhotosPosture, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ArmGrasping, std::bind(&Robot::Unload_ArmGrasping, this)));
    umapWorkFun.insert(std::make_pair(RobotState::ArmUploading, std::bind(&Robot::ArmUploading, this)));    
    umapWorkFun.insert(std::make_pair(RobotState::Unload_BoxIdentification, std::bind(&Robot::Unload_BoxIdentification, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ToPlatformPoint, std::bind(&Robot::Unload_ToPlatformPoint, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ToWaitPoint, std::bind(&Robot::Unload_ToWaitPoint, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ToMissionPlatformPoint, std::bind(&Robot::Unload_ToMissionPlatformPoint, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_PlatformPointMove, std::bind(&Robot::Unload_PlatformPointMove, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ManualMount, std::bind(&Robot::Unload_ManualMount, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ArmToRangingPosture, std::bind(&Robot::Unload_ArmToRangingPosture, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ArmToRangingPosture_1, std::bind(&Robot::Unload_ArmToRangingPosture_1, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ArmToRangingPosture_2, std::bind(&Robot::Unload_ArmToRangingPosture_2, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ArmToRangingPosture_3, std::bind(&Robot::Unload_ArmToRangingPosture_3, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ToWorkPoint, std::bind(&Robot::Unload_ToWorkPoint, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_ManualUnMount, std::bind(&Robot::Unload_ManualUnMount, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Unload_OpenCamera, std::bind(&Robot::Unload_OpenCamera, this)));
    // 装货 流程

    // 装货流程 by JF Gan 
    umapWorkFun.insert(std::make_pair(RobotState::Load_ToMissionPosition, std::bind(&Robot::Load_ToMissionPosition, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_ArmToPhotosPose1, std::bind(&Robot::Load_ArmToPhotosPose1, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_ContainerOffset, std::bind(&Robot::Load_ContainerOffset, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_ArmOrigin, std::bind(&Robot::Load_ArmOrigin, this))); 
    umapWorkFun.insert(std::make_pair(RobotState::Load_ToOffsetPosition, std::bind(&Robot::Load_ToOffsetPosition, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_ManualStartLoad, std::bind(&Robot::Load_ManualStartLoad, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_ToLoadPosition, std::bind(&Robot::Load_ToLoadPosition, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_ArmToPhotosPose2, std::bind(&Robot::Load_ArmToPhotosPose2, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_ComputeBoxLoadPose, std::bind(&Robot::Load_ComputeBoxLoadPose, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_BoxLoading, std::bind(&Robot::Load_BoxLoading, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_ManualStopLoad, std::bind(&Robot::Load_ManualStopLoad, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_ToPlatformPoint, std::bind(&Robot::Load_ToPlatformPoint, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_ManualStopMission, std::bind(&Robot::Load_ManualStopMission, this)));
    umapWorkFun.insert(std::make_pair(RobotState::Load_ArmToRangingPosture, std::bind(&Robot::Load_ArmToRangingPosture, this)));

    // Create thread
    thWork_ = std::thread(&Robot::RobotWork, this);
    pTaskManagement_ = new TaskManagement::TaskManagement(std::bind(&Robot::TaskCallBack, this, std::placeholders::_1));
    pRobotChassis_ = new RobotChassis::RobotChassis(std::bind(&Robot::ChassisStateCallBack, this, std::placeholders::_1));
    pRobotArm_ = new RobotArm::RobotArm(std::bind(&Robot::RobotArmStateCallBack, this, std::placeholders::_1));
    pVisualIdentity_ = new VisualIdentity::VisualIdentity(std::bind(&Robot::VisualIdentityStateCallBack, this, std::placeholders::_1));
    return true;
}


void Robot::Stop()
{
    bIsStop_ = true;
}


void Robot::RobotWork()
{
    while (!bIsStop_)
    {
        if (bRobotPause && robot_state_ != PauseBackup_RobotState_)
        {
            RobotPauseStateBackup();
        }
        pRobotStateData_->robot_state_ = robot_state_;
        if (pUpdateCallback(pRobotStateData_->GetObject()))
        {
            pRobotStateData_->bTaskStateUpdated_ = false;
            pRobotStateData_->bChassisStateUpdated_ = false;
            pRobotStateData_->bRobotArmStateUpdated_ = false;
            pRobotStateData_->bVisualIdentityStateUpdated_ = false;
        }

        umapWorkFun[robot_state_]();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


void Robot::SetUpdateCallBack(UPDATE_CALLBACK updata_callback)
{
    pUpdateCallback = updata_callback;
}


//  未初始化
bool Robot::Uninitialized()
{
    if (pRobotStateData_->TaskStateData_.TaskState_ != TaskManagement::TaskState::Normal)
    {   // 任务模块状态非正常
        return false;
    }
    if (pRobotStateData_->ChassisStateData_.ChassisState_ != RobotChassis::ChassisState::NetworkNormal)
    {   // 底盘状态非正常
        return false;
    }
    if (pRobotStateData_->VisualIdentityStateData_.VisualIdentityState_ != VisualIdentity::VisualIdentityState::Normal)
    {   // 视觉识别状态非正常
        return false;
    }
//    if (pRobotStateData_->RobotArmStateData_.RobotArmState_ != RobotArm::RobotArmState::Normal)
//    {   // 机械臂状态非正常
//        return false;
//    }
    robot_state_ = RobotState::Ready;
    qDebug() << "Ready";
    return true;
}

//  准备就绪
bool Robot::Ready()
{
    TaskManagement::TaskData temp_task_data;
    if (pTaskManagement_->GetANewTask(temp_task_data))
    {
//        bArmArrive_ = false;
//        pRobotArm_->ToFoldingPosture();

        if (temp_task_data.iCarriageId_ != pRobotStateData_->ChassisStateData_.iCurrentStationID_)
        {
            bNavArrive_ = false;
            pRobotChassis_->SendGoal(temp_task_data.iCarriageId_);
        }
        else
        {
            bNavArrive_ = true;
        }

        if (temp_task_data.iWorkType_ == 0) // 0:unload  1:load
        {
            robot_state_ = RobotState::Unload_ToMissionPlatformPoint;
            qDebug() << "Unload_ToMissionPlatformPoint";
        }
        else if (temp_task_data.iWorkType_ == 1)
        {
            robot_state_ = RobotState::Load_ToMissionPosition;
            qDebug() << "Load_ToMissionPosition";
        }
        else
        {
            return false;
        }

        pRobotArm_->TcpHoming();
//        PlatformPoint_ = temp_task_data.Pose_;
    }
    return true;
}

// ==================卸货========================
//  前往任务点
bool Robot::Unload_ToMissionPoint()
{
    if (bNavArrive_)
    {   // 底盘到达目的
        bNavArrive_ = false;
        bArmArrive_ = false;
        pRobotArm_->ToPhotosPosture(iIdentificationRegion_); // lucien - todo
        robot_state_ = RobotState::Unload_ArmToPhotosPosture;
        qDebug() << "Unload_ArmToPhotosPosture";
    }
    return true;
}

//  机械臂到拍照姿势
bool Robot::Unload_ArmToPhotosPosture()
{
    qDebug() << "ArmToPhotosPosture";
    if (bArmArrive_)
    {
        bArmArrive_ = false;
#if 0
        double dToBoxDis = pRobotArm_->GetPointLaserShortDistance() / 1000.0;
        dToBoxDis = 2500;      // 模拟测试需要修改
        if (dToBoxDis < 2200)
        {
            bNavArrive_ = false;
            if (pRobotArm_->GetPointLaserLeftDistance() < 969)
            {
                if (pRobotArm_->GetPointLaserLeftDistance() > 812)  //
                {
                    pRobotChassis_->QuantitativeReceding((2500 - dToBoxDis), pRobotArm_->GetPointLaserLeftDistance() - 812);
                }
                else if (pRobotArm_->GetPointLaserLeftDistance() < 812)
                {
                    pRobotChassis_->QuantitativeReceding((2500 - dToBoxDis), -(812 - pRobotArm_->GetPointLaserLeftDistance()));
                }
            }
            else
            {
                pRobotChassis_->QuantitativeReceding((2500 - dToBoxDis), 0.0);
            }
            robot_state_ = RobotState::Unload_ToMissionPoint;
        }
        else if (dToBoxDis > 2800)
        {
            bNavArrive_ = false;
            if (pRobotArm_->GetPointLaserLeftDistance() < 969)
            {
                if (pRobotArm_->GetPointLaserLeftDistance() > 812)  //
                {
                    pRobotChassis_->QuantitativeAdvance((dToBoxDis - 2500), pRobotArm_->GetPointLaserLeftDistance() - 812);
                }
                else if (pRobotArm_->GetPointLaserLeftDistance() < 812)
                {
                    pRobotChassis_->QuantitativeAdvance((dToBoxDis - 2500), -(812 - pRobotArm_->GetPointLaserLeftDistance()));
                }
            }
            else
            {
                pRobotChassis_->QuantitativeAdvance((dToBoxDis - 2500), 0.0);
            }
            robot_state_ = RobotState::Unload_ToMissionPoint;
        }
        else
        {   // 满足视觉识别的需求
            pRobotArm_->IoOut(8, true);
            pRobotArm_->IoOut(9, true);
            llWaitTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            robot_state_ = RobotState::Unload_OpenCamera;
        }
#else
        pRobotArm_->IoOut(8, true);
        pRobotArm_->IoOut(9, true);
        llWaitTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        robot_state_ = RobotState::Unload_OpenCamera;
#endif
    }
    return true;
}

//  机械臂抓取
bool Robot::Unload_ArmGrasping()
{
    qDebug() << "ArmGrasping";
    if (bArmArrive_)
    {
        bArmArrive_ = false;
//        pRobotArm_->IoOut(8, true);
//        pRobotArm_->IoOut(9, true);
        
        // 抓取完成
        if (iIdentificationRegion_ == 2)
        {
            pRobotArm_->ToPhotosPosture(iIdentificationRegion_); // lucien - todo
            robot_state_ = RobotState::Unload_ArmToPhotosPosture;

            std::stringstream log_data;
            log_data << u8"抓取完成 前往下拍照点 ";
            log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);
        }
        else
        {   // 测距 和 移动底盘
            bArmArrive_ = false;
            pRobotArm_->ToRangingPosture(0);
            robot_state_ = RobotState::Unload_ArmToRangingPosture_1;
            std::stringstream log_data;
            log_data << u8"抓取完成 前往测距点 ";
            log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);
        }

    }
    return true;
}

//  机械臂装货
bool Robot::ArmUploading()
{
    qDebug() << "ArmUploading";
    if (bArmArrive_)
    {
        bArmArrive_ = false;
        pRobotArm_->ToPhotosPosture(iIdentificationRegion_); // lucien - todo
        robot_state_ = RobotState::Upload_ArmToPhotosPosture;
    }
    return true;
}

bool Robot::Unload_OpenCamera()
{
    long long time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if ((time - llWaitTime_) > 3)
    {
        std::stringstream log_data;
        log_data << u8"启动拍照识别 当前拍照点 ： " << iIdentificationRegion_;
        log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);

        bVisualFulfil_ = false;
        pRobotStateData_->VisualIdentityStateData_.vecBoxPose_.clear();
        pVisualIdentity_->ActivationIdentification(iIdentificationRegion_);
        iIdentificationRegion_ = iIdentificationRegion_ == 1 ? 2 : 1;
        iConfirmVisualResultsMode_ = 0;
        robot_state_ = RobotState::Unload_BoxIdentification;
        qDebug() << "Unload_BoxIdentification";
    }
    else
    {
        qDebug() << time - llWaitTime_;
    }
    return true;
}

//  箱体识别
bool Robot::Unload_BoxIdentification()
{
    // qDebug() << "Unload_BoxIdentification";
    if (bVisualFulfil_)
    {
        if (iConfirmVisualResultsMode_ == 0)
        {
            return true;
        }
        else if (iConfirmVisualResultsMode_ == 2) // 重拍
        {
            bVisualFulfil_ = false;
            pRobotStateData_->VisualIdentityStateData_.vecBoxPose_.clear();
            pVisualIdentity_->ActivationIdentification((iIdentificationRegion_ == 1 ? 2 : 1));
            iConfirmVisualResultsMode_ = 0;

            std::stringstream log_data;
            log_data << u8"重新拍照 当前拍照点 ： " << (iIdentificationRegion_ == 1 ? 2 : 1);
            log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);

            return true;
        }

        bVisualFulfil_ = false;
        pRobotArm_->IoOut(8, false);
        pRobotArm_->IoOut(9, false);

        if (pRobotStateData_->VisualIdentityStateData_.vecBoxPose_.size() == 0 &&
            pRobotStateData_->VisualIdentityStateData_.vecUpLoadBoxPose_.size() == 0
            && iIdentificationRegion_ == 1) //在机械臂完成后，会在状态回调函数中更改该值，所以当前如果是1，就代表刚刚图像识别的是2动作
        {
            // 2动作是最底层的位置，如果这个位置识别不到，说明全货厢卸货完成
            bNavArrive_ = false;
            pRobotChassis_->SendGoal(iAdjustmentPointID_); //todo
            robot_state_ = RobotState::Unload_ToPlatformPoint;

            std::stringstream log_data;
            log_data << u8"下拍照点 没有识别出箱体  回到月台 " << iAdjustmentPointID_;
            log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);

            return true;
        }
        if (pRobotStateData_->VisualIdentityStateData_.vecBoxPose_.size() == 0 &&
                pRobotStateData_->VisualIdentityStateData_.vecUpLoadBoxPose_.size() == 0)
        {
            bArmArrive_ = false;
            pRobotArm_->ToPhotosPosture(iIdentificationRegion_); // lucien - todo
            robot_state_ = RobotState::Unload_ArmToPhotosPosture;

            std::stringstream log_data;
            log_data << u8"上拍照点 没有识别出箱体 在下拍照点再拍一张 ";
            log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);

            return true;
        }
        bArmArrive_ = false;
        pRobotArm_->GraspingBox(pRobotStateData_->VisualIdentityStateData_.vecBoxPose_, (iIdentificationRegion_ == 1 ? 2 : 1));
        robot_state_ = RobotState::Unload_ArmGrasping;

        std::stringstream log_data;
        log_data << u8"识别到箱体  数量：" << pRobotStateData_->VisualIdentityStateData_.vecBoxPose_.size();
        log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);
//        if(pRobotStateData_->VisualIdentityStateData_.iwork_mode_ == 1)//卸货模式
//        {
//            pRobotArm_->GraspingBox(pRobotStateData_->VisualIdentityStateData_.vecBoxPose_);
//            robot_state_ = RobotState::Unload_ArmGrasping;
//        }
//        if(pRobotStateData_->VisualIdentityStateData_.iwork_mode_ == 2)
//        {
//            pRobotArm_->UploadingBox(pRobotStateData_->VisualIdentityStateData_.vecUpLoadBoxPose_);
//            robot_state_ = RobotState::ArmUploading;
//        }
    }
    return true;
}

//  前往月台点
bool Robot::Unload_ToPlatformPoint()
{
    qDebug() << "Unload_ToPlatformPoint";
    if (bNavArrive_)
    {
        if (bCompleteUnMount_)
        {   // 人工卸下挂载
            bCompleteUnMount_ = false;
            bNavArrive_ = false;
            TaskManagement::TaskData temp_task_data;
            if (pTaskManagement_->GetANewTask(temp_task_data))
            {
                pRobotChassis_->SendGoal(temp_task_data.iCarriageId_);
                if (temp_task_data.iWorkType_ == 0) // 0:unload  1:load
                {
                    iPlatformPointID_ = temp_task_data.iCarriageId_;
                    iAdjustmentPointID_ = temp_task_data.iCarriageId_;
                    robot_state_ = RobotState::Unload_ToMissionPlatformPoint;
                }
                else if (temp_task_data.iWorkType_ == 1)
                {
                    robot_state_ = RobotState::Load_ToMissionPosition;
                }
                else
                {
                    pRobotChassis_->SendGoal(iWaitPointID_);
                    robot_state_ = RobotState::Unload_ToWaitPoint;
                }
            }
            else
            {
                pRobotChassis_->SendGoal(iWaitPointID_);
                robot_state_ = RobotState::Unload_ToWaitPoint;
            }
        }
    }
    return true;
}

//  前往等待点
bool Robot::Unload_ToWaitPoint()
{
    qDebug() << "ToWaitPoint";
    if (bNavArrive_)
    {
        bNavArrive_ = false;
        // pRobotArm_->ToPhotosPosture(iIdentificationRegion_); // lucien - todo
        robot_state_ = RobotState::Ready;
    }
    return true;
}

// 前往任务月台点
bool Robot::Unload_ToMissionPlatformPoint()
{
    qDebug() << u8"前往任务月台点";
    if (bNavArrive_)
    {   // 底盘到达目的
        bNavArrive_ = false;
#if 0
        // 左右偏移 - 目的是与集装箱保持平行
//        pRobotArm_->GetPointLaserDistance();
        int temp_station = -1;
        if (pRobotArm_->GetPointLaserLeftDistance() < 969)
        {
            if (pRobotArm_->GetPointLaserLeftDistance() > 812)  //
            {
                temp_station = pRobotChassis_->QuantitativeMove(0.0, 0.0, pRobotArm_->GetPointLaserLeftDistance() - 812);  // Todo
            }
            else if (pRobotArm_->GetPointLaserLeftDistance() < 812)
            {
                temp_station = pRobotChassis_->QuantitativeMove(0.0, 0.0, -(812 - pRobotArm_->GetPointLaserLeftDistance()));  // Todo
            }
        }
        else
        {
            bCompleteMount_ = false;
            llWaitTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            robot_state_ = RobotState::Unload_ManualMount;
            return true;
        }

        if (temp_station != -1)
        {
            iPlatformPointID_ = temp_station;
            iAdjustmentPointID_ = temp_station;
        }
        robot_state_ = RobotState::Unload_PlatformPointMove;
        qDebug() << u8"人工挂载";
#else
    // 不进行偏移
        bCompleteMount_ = false;
        llWaitTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        robot_state_ = RobotState::Unload_ManualMount;
        qDebug() << u8"人工挂载";
#endif

    }
    return true;
}

//bool Robot::Unload_ArmToPhotosPosture_Offset()
//{
//    if (bArmArrive_)
//    {
//        bArmArrive_ = false;
//    }
//    return true;
//}

// 月台点移动
bool Robot::Unload_PlatformPointMove()
{
    qDebug() << u8"月台点移动";
    if (bNavArrive_)
    {   // 底盘到达目的
        bNavArrive_ = false;
        // PlatformPoint_
        // 需要人工挂载下货设备
        bCompleteMount_ = false;
        llWaitTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        robot_state_ = RobotState::Unload_ManualMount;
    }
    return true;
}

// 人工挂载
bool Robot::Unload_ManualMount()
{
    qDebug() << u8"人工挂载";
    if (bCompleteMount_)
    {
//        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count()
//            - llWaitTime_ < 2)
//        {
//            return true;
//        }
        llWaitTime_ = ULLONG_MAX;
        bCompleteMount_ = false;
        // 机械臂 移动到测距离姿势
        bArmArrive_ = false;
        pRobotArm_->ToRangingPosture(0);
//        llWaitTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        llWaitTime_ = 0.0;
        robot_state_ = RobotState::Unload_ArmToRangingPosture_1;
        qDebug() << u8"机械臂到测距姿势";
    }
    return true;
}

// 机械臂到测距姿势
bool Robot::Unload_ArmToRangingPosture_1()
{
    // 判断是否到位
    if (bArmArrive_)
    {
        if (llWaitTime_ < 1)
        {
            llWaitTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        }
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count()
                - llWaitTime_ < 1) // 等待1秒
        {
            return true;
        }
        dRangingEsult_[0] = pRobotArm_->GetPointLaserLongDistance();
        bArmArrive_ = false;
        pRobotArm_->ToRangingPosture(1);
        llWaitTime_ = 0.0;
        robot_state_ = RobotState::Unload_ArmToRangingPosture_2;
    }
    return true;
}

// 机械臂到测距姿势
bool Robot::Unload_ArmToRangingPosture_2()
{
    // 判断是否到位
    if (bArmArrive_)
    {
        if (llWaitTime_ < 1)
        {
            llWaitTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        }
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count()
                - llWaitTime_ < 1) // 等待1秒
        {
            return true;
        }
        dRangingEsult_[1] = pRobotArm_->GetPointLaserLongDistance();
        bArmArrive_ = false;
        pRobotArm_->ToRangingPosture(2);
        llWaitTime_ = 0.0;
        robot_state_ = RobotState::Unload_ArmToRangingPosture_3;
    }
    return true;
}

// 机械臂到测距姿势
bool Robot::Unload_ArmToRangingPosture_3()
{
    // 判断是否到位
    if (bArmArrive_)
    {
        if (llWaitTime_ < 1)
        {
            llWaitTime_ = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        }
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count()
                - llWaitTime_ < 1) // 等待1秒
        {
            return true;
        }
        dRangingEsult_[2] = pRobotArm_->GetPointLaserLongDistance();
        bArmArrive_ = false;
        pRobotArm_->ToRangingPosture(0);
        llWaitTime_ = 0.0;
        robot_state_ = RobotState::Unload_ArmToRangingPosture;
    }
    return true;
}

// 机械臂到测距姿势
bool Robot::Unload_ArmToRangingPosture()
{
//    qDebug() << u8"机械臂到测距姿势";
    if (bArmArrive_)
    {
        double dToBoxDis = 9999.0;
        for (int i = 0; i < 3; i++)
        {
            if (dRangingEsult_[i] < dToBoxDis)
            {
                dToBoxDis = dRangingEsult_[i];  // 取最小的值
            }
        }
        bArmArrive_ = false;
//        double dToBoxDis = pRobotArm_->GetPointLaserLongDistance();
//        qDebug() << "距离" << dToBoxDis;
//        dToBoxDis = 300;
        if (dToBoxDis < 249)    //  M
        {
            bNavArrive_ = false;
#if 0
            if (pRobotArm_->GetPointLaserLeftDistance() < 969)
            {
                if (pRobotArm_->GetPointLaserLeftDistance() > 812)  //
                {
                    pRobotChassis_->QuantitativeMove(-180.0, 2.5 - dToBoxDis, pRobotArm_->GetPointLaserLeftDistance() - 812);  // Todo
                }
                else if (pRobotArm_->GetPointLaserLeftDistance() < 812)
                {
                    pRobotChassis_->QuantitativeMove(-180.0, 2.5 - dToBoxDis, -(812 - pRobotArm_->GetPointLaserLeftDistance()));  // Todo
                }
            }
            else
            {
                pRobotChassis_->QuantitativeMove(-180.0, 2.5 - dToBoxDis, 0.0);  //向前0° - 底盘的前与机器人的前相反
            }
#else
            std::stringstream log_data;
            log_data << u8"测距移动 距离为：" << dToBoxDis << "  向后移动 目标点:";
            log_data << pRobotChassis_->QuantitativeMove(0.0, 250 - dToBoxDis, 0.0);  //向前0° - 底盘的前与机器人的前相反
            log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);
#endif

            robot_state_ = RobotState::Unload_ToMissionPoint;
        }
        else if (dToBoxDis > 255)   // M
        {
            bNavArrive_ = false;
#if 0
            if (pRobotArm_->GetPointLaserLeftDistance() < 969)
            {
                if (pRobotArm_->GetPointLaserLeftDistance() > 812)  //
                {
                    pRobotChassis_->QuantitativeMove(0.0, dToBoxDis - 2.5, pRobotArm_->GetPointLaserLeftDistance() - 812);  // Todo
                }
                else if (pRobotArm_->GetPointLaserLeftDistance() < 812)
                {
                    pRobotChassis_->QuantitativeMove(0.0, dToBoxDis - 2.5, -(812 - pRobotArm_->GetPointLaserLeftDistance()));  // Todo
                }
            }
            else
            {
                pRobotChassis_->QuantitativeMove(0.0, dToBoxDis - 2.5, 0.0);   //向后180° - 底盘的前与机器人的前相反
            }
#else
            std::stringstream log_data;
            log_data << u8"测距移动 距离为：" << dToBoxDis << "  向前移动 目标点:";
            log_data << pRobotChassis_->QuantitativeMove(-180.0, dToBoxDis - 250, 0.0);   //向后180° - 底盘的前与机器人的前相反
            log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);
#endif
            robot_state_ = RobotState::Unload_ToMissionPoint;
        }
        else
        {   // 满足视觉识别的需求

            std::stringstream log_data;
            log_data << u8"满足视觉识别的需求 距离值：" << dToBoxDis << "前往拍照点:" << (iIdentificationRegion_ == 1 ? u8"上" : u8"下");
            log_.AddLog(__LINE__, __FILE__, log_data.str().c_str(), log_data.str().size(), LLOG::TLogLevel::DEBUG);

            pRobotArm_->ToPhotosPosture(iIdentificationRegion_); // lucien - todo
            robot_state_ = RobotState::Unload_ArmToPhotosPosture;
        }
    }
    return true;
}

// 前往工作点
bool Robot::Unload_ToWorkPoint()
{
    qDebug() << u8"前往工作点";
    if (bNavArrive_)
    {   // 底盘到达目的
        bNavArrive_ = false;
        bArmArrive_ = false;
        pRobotArm_->ToRangingPosture();
        robot_state_ = RobotState::Unload_ArmToRangingPosture;
    }
    return true;
}

// 人工卸载
bool Robot::Unload_ManualUnMount()
{
    return true;
}

// ==================卸货 END========================

//  暂停
bool Robot::Pause()
{
    qDebug() << "Pause";
    // 保存当前状态
//    机械臂暂停函数

    return true;
}

//  恢复
bool Robot::Resume()
{
    qDebug() << "Resume";
//    RobotArm::RobotArmControl control_data;
//    control_data.ControlType_ = RobotArm::RobotArmControl::ControlType::ResumeAllAxis;
//    pRobotArm_->ResumeToRun(control_data);
    return true;
}

// 任务模块回调函数
void Robot::TaskCallBack(const TaskManagement::TaskStateData& data)
{
    if (pRobotStateData_ != nullptr)
    {
        *pRobotStateData_ = data;
    }
}

// 底盘状态回调函数
void Robot::ChassisStateCallBack(const RobotChassis::ChassisStateData& data)
{
    if (pRobotStateData_ != nullptr)
    {
        *pRobotStateData_ = data;
        if (data.NavigationState_ == RobotChassis::NavigationState::Arrive)
        {
            bNavArrive_ = true;
        }
    }
}

// 机械臂状态回调函数
void Robot::RobotArmStateCallBack(const RobotArm::RobotArmStateData& data)
{
    if (pRobotStateData_ != nullptr)
    {
        *pRobotStateData_ = data;
        if (data.ExecuteState_ == RobotArm::ExecuteState::Unload_Arrive
                || data.ExecuteState_ == RobotArm::ExecuteState::Upload_Arrive
                || data.ExecuteState_ == RobotArm::ExecuteState::Arrive)
        {
            bArmArrive_ = true;
        }

//        if (data.ExecuteState_ == RobotArm::ExecuteState::Fail || data.ExecuteState_ == RobotArm::ExecuteState::Fail)
//        {
//            // 默认移动到拍照点是不会失败的
//            pRobotArm_->ToPhotosPosture(iIdentificationRegion_);
//            robot_state_ = RobotState::Unload_ArmToPhotosPosture;
//        }
    }
}

// 视觉识别状态回调函数
void Robot::VisualIdentityStateCallBack(const VisualIdentity::VisualIdentityStateData& data)
{
    if (pRobotStateData_ != nullptr)
    {
        *pRobotStateData_ = data;
    }
    if (data.VisualIdentityState_ == VisualIdentity::VisualIdentityState::ERROR)
    {
        std::string tempStr(u8"无法读取到深度相机");
        pErrorCallback_(tempStr);
        return;
    }
    bVisualFulfil_ = true;
}

//机器人暂停运行
bool Robot::RobotPause()
{
    if (robot_state_ == RobotState::Pause)
    {   // 当前已经是 暂停 模式
        return false;
    }
    // 先让机器人停止
    // 保存当前机器人的运行状态
    if (pRobotArm_) // 机械臂 暂停
    {
        pRobotArm_->Pause();
    }
    if (pRobotChassis_) // 底盘 暂停
    {
        pRobotChassis_->Pause();
    }
    RobotPauseStateBackup();    // 记录 当前的状态
    robot_state_ = RobotState::Pause;
    return true;
}

void Robot::RobotPauseStateBackup()
{
    PauseBackup_RobotState_ = robot_state_; // 记录 当前的状态
}

bool Robot::RobotReset()
{
    if (pRobotArm_) // 机械臂 暂停
    {
        pRobotArm_->Reset();
    }
    robot_state_ = RobotState::Uninitialized;
    return true;
}

//机器人恢复运行
bool Robot::RobotResumeToRun()
{
    if (robot_state_ == RobotState::Pause)
    {
//        robot_state_ = RobotState::Resume;
        robot_state_ = PauseBackup_RobotState_;
        // 判断当前状态是不是 等待状态
        if (pRobotArm_) // 机械臂 暂停
        {
            pRobotArm_->ResumeToRun();
        }
        if (pRobotChassis_) // 底盘 暂停
        {
            pRobotChassis_->Pause();
        }
        return true;
    }
    return false;
}

// 完成人工挂载
bool Robot::CompleteMounting()
{
   bManualStartLoad_ = true;
   if (robot_state_ == RobotState::Unload_ManualMount)
   {
       if (pRobotStateData_->RobotArmStateData_.RobotArmState_ == RobotArm::RobotArmState::Normal)
       {
           bCompleteMount_ = true;
       }
       else
       {
           std::string tempStr(u8"机械臂状态异常");
           pErrorCallback_(tempStr);
           return false;
       }
   }
   else
   {
       std::string tempStr(u8"机械臂通讯异常");
       pErrorCallback_(tempStr);
       return false;
   }
    return true;
}

bool Robot::CompleteUnMounting()
{
    bManualStopMission_ = true;
    if ((robot_state_ == RobotState::Unload_ManualUnMount)
        || (robot_state_ == RobotState::Unload_ToPlatformPoint))
    {
        bCompleteUnMount_ = true;
    }
    else
    {
        return false;
    }
    return true;
}

void Robot::SetConfirmVisualResultsMode(int mode_id)
{
    iConfirmVisualResultsMode_ = mode_id;
}

//  装货流程相关函数 by JF Gan

//  状态：前往任务点（装货）
bool Robot::Load_ToMissionPosition()
{
    qDebug() << "前往任务点";
    if (bNavArrive_ && bArmArrive_)
    {   // 底盘到达目的
        bNavArrive_ = false;
        bArmArrive_ = false;
//        iIdentificationRegion_ = 101;
//        pRobotArm_->ToPhotosPosture(2); // test 101;
//        robot_state_ = RobotState::Load_ArmToPhotosPose1;
        pRobotArm_->ToRangingPosture(); // 测距姿态
        robot_state_ = RobotState::Load_ArmOrigin;  // 现在版本没有测量偏移

    }
    return true;
}

//  状态：机械臂拍照位姿1（计算车厢偏移距离）
bool Robot::Load_ArmToPhotosPose1()
{
    qDebug() << "Load_ArmToPhotosPose1";
    if (bArmArrive_)
    {
        bArmArrive_ = false;
        pVisualIdentity_->CompteContainerOffset();
        robot_state_ = RobotState::Load_ContainerOffset;
    }
    return true;
}

//  状态：正在计算当前车厢偏移距离
bool Robot::Load_ContainerOffset()
{
    qDebug() << "Load_ContainerOffset";
    if (bVisualFulfil_)
    {   // 获取到视觉结果，但未折叠
        bVisualFulfil_ = false;
        // 此处是否判断视觉计算值的合理性，待讨论
        bArmArrive_ = false;
        pRobotArm_->ToFoldingPosture();
        robot_state_ = RobotState::Load_ArmOrigin;
    }
    return true;
}

//  状态：机械臂回零
bool Robot::Load_ArmOrigin()
{
    qDebug() << "Load_ArmOrigin";
    if (bArmArrive_)
    {
        bArmArrive_ = false;
        if(bManualStopLoad_)
        {
            bManualStopLoad_ = false;
            bNavArrive_ = false;
            pRobotChassis_->SendGoal(iPlatformPointID_);
            robot_state_ = RobotState::Load_ToPlatformPoint;
        }
        else
        {
//            bNavArrive_ = false;
            // if (false)  // 模拟测试需要修改
//            if (pRobotArm_->GetPointLaserLeftDistance() < 969) // 判断是不是在集装箱内
//            {
//                if (pRobotArm_->GetPointLaserLeftDistance() > 812)  //
//                {
//                    pRobotChassis_->QuantitativeMove(0.0, 0.0, pRobotArm_->GetPointLaserLeftDistance() - 812);  // Todo
//                }
//                else if (pRobotArm_->GetPointLaserLeftDistance() < 812)
//                {
//                    pRobotChassis_->QuantitativeMove(0.0, 0.0, -(812 - pRobotArm_->GetPointLaserLeftDistance()));  // Todo
//                }
//            }
//            else
//            {
//                iPlatformPointID_ = pRobotChassis_->QuantitativeMove(-90.0, -pRobotStateData_->VisualIdentityStateData_.dValue_, 0.0);
//            }
            robot_state_ = RobotState::Load_ToOffsetPosition;
            // if(pRobotStateData_->VisualIdentityStateData_.XXXX > 10)
            //     iPlatformPointID_ = pRobotChassis_->QuantitativeMove(VisualIdentityStateData_.XXXX, VisualIdentityStateData_.XXXX); 
            //     robot_state_ = RobotState::Load_ToOffsetPosition;   
            // else
            //     robot_state_ = RobotState::Load_ManualStartLoad;            
        } 
    }
    return true;
}

//  状态：移动车厢偏移距离到车厢中心线上
bool Robot::Load_ToOffsetPosition()
{
    qDebug() << "Load_ToOffsetPosition";
    if (bNavArrive_)
    {
        bNavArrive_ = false;
        robot_state_ = RobotState::Load_ManualStartLoad;
    }
    return true;
}

//  状态：人工启动装货
bool Robot::Load_ManualStartLoad()
{
    qDebug() << "Load_ManualStartLoad";
    if (bManualStartLoad_)
    {
        bManualStartLoad_ = false;
        // 获得货箱尺寸
//        pRobotArm_->GetBoxSize(&CurrentBox_);
        CurrentBox_.Length = 500;
        CurrentBox_.Width = 340;
        CurrentBox_.Height = 120;
        bArmArrive_ = false;
        pRobotArm_->ToRangingPosture();
        robot_state_ = RobotState::Load_ComputeBoxLoadPose;    // 机械臂移动到测距姿态
    }
    return true;
}

// 状态：正在前往装货位置
bool Robot::Load_ToLoadPosition()
{
    if (bNavArrive_)
    {
        bNavArrive_ = false;
        bArmArrive_ = false;
        pRobotArm_->ToRangingPosture();
        robot_state_ = RobotState::Load_ArmToRangingPosture;    // 机械臂移动到测距姿态
//        double CurrentDistance = pRobotArm_->GetPointLaserDistance() * 1000;
        // if(CurrentDistance <= 200 + CurrentBox_.Length - LoadOffset)
        // {
        //     pRobotChassis_->QuantitativeReceding(200 + CurrentBox_.Length-CurrentDistance); 
        //     robot_state_ = RobotState::Load_ToLoadPosition;
        // } 
        // else if( CurrentDistance >= 200 + CurrentBox_.Length + LoadOffset)
        // {
        //     pRobotChassis_->QuantitativeAdvance(200 + CurrentBox_.Length-CurrentDistance);
        //     robot_state_ = RobotState::Load_ToLoadPosition;
        // }    
        // else if(200 + CurrentBox_.Length - LoadOffset < CurrentDistance < 200 + CurrentBox_.Length + LoadOffset)
        // {
        //     iIdentificationRegion_ = 2;
        //     pRobotArm_->ToPhotosPosture(iIdentificationRegion_);
        //     robot_state_ = RobotState::Load_ArmToPhotosPose2;    
        // }       
    }
    return true;
}

// 状态：机械臂运动到拍照位姿2
bool Robot::Load_ArmToPhotosPose2()
{

    if (bArmArrive_)
    {
        bArmArrive_ = false;
        robot_state_ = RobotState::Load_ComputeBoxLoadPose;
        qDebug() << "Load_ComputeBoxLoadPose";
    }
    return true;
}

// 状态：正在计算货箱放置位置
bool Robot::Load_ComputeBoxLoadPose()
{

    CurrentContainer_.dInsideLength_ = 6000;
    CurrentContainer_.dInsideWidth_ = 2200; // 2280 测试需要保守一点
    CurrentContainer_.dInsideHeight_ = 2200;    // 2200 测试需要保守一点
    bArmArrive_ = false;
    pRobotArm_->UploadingBox(CurrentContainer_, CurrentBox_);
    robot_state_ = RobotState::Load_BoxLoading;
    qDebug() << "Load_BoxLoading";


//    if (bVisualFulfil_)
//    {
//        bVisualFulfil_ = false;
////         pRobotArm_->LoadBox(pRobotStateData_->VisualIdentityStateData_.vecUpLoadBoxPose_);
////        bArmArrive_ = false;
////        pRobotArm_->UploadingBox(pRobotStateData_->VisualIdentityStateData_.vecUpLoadBoxPose_);
//        // 获取激光数据
//        // 根据 集装箱大小 计算右下角的位置
//        // 计算出一面箱体的安装位置

////        Common::Container container(6000, 2000, 2100);
////        Common::Box current_box(500, 400, 300);

//    }
    return true;
}

// 状态：正在放置货箱
bool Robot::Load_BoxLoading()
{
    if (bArmArrive_)
    {
        // 装载完成 底盘需要往后退 一个箱子的深度
        bArmArrive_ = false;
        bNavArrive_ = false;
        pRobotChassis_->QuantitativeReceding(CurrentBox_.Length, 0);
        robot_state_ = RobotState::Load_LoadingDone_MoveBackward;
    }
    return true;
}

// 状态：人工结束本次装货
bool Robot::Load_ManualStopLoad()
{
    qDebug() << "Load_ManualStopLoad";
    if (bManualStopLoad_)
    {
        // bManualStopLoad_ = false;
        // pRobotArm_->BackToOrigin(); 
        bArmArrive_ = false;
        pRobotArm_->ToFoldingPosture();
        robot_state_ = RobotState::Load_ArmOrigin;
    }
    return true;
}

// 状态：前往装货月台点
bool Robot::Load_ToPlatformPoint()
{
    qDebug() << "Load_ToPlatformPoint";
//    if (bManualStopLoad_)
//    {
//        robot_state_ = RobotState::Load_ManualStopMission;
//    }
    if (bNavArrive_)
    {
        bNavArrive_ = false;
        bManualStopMission_ = false;
        robot_state_ = RobotState::Load_ManualStopMission;
    }
    return true;
}

// 状态：人工结束本次任务
bool Robot::Load_ManualStopMission()
{
    qDebug() << "Load_ManualStopMission";
    if (bManualStopMission_)
    {
        bManualStopMission_ = false;
        TaskManagement::TaskData temp_task_data;
        if (pTaskManagement_->GetANewTask(temp_task_data))
        {
            pRobotChassis_->SendGoal(temp_task_data.Pose_);
//            robot_state_ = RobotState::Load_ToMissionPosition;
            PlatformPoint_ = temp_task_data.Pose_;

            if (temp_task_data.iWorkType_ == 0) // 0:unload  1:load
            {
                robot_state_ = RobotState::Unload_ToMissionPlatformPoint;
            }
            else if (temp_task_data.iWorkType_ == 1)
            {
                robot_state_ = RobotState::Load_ToMissionPosition;
            }
            else
            {
                pRobotChassis_->SendGoal(iWaitPointID_);
                robot_state_ = RobotState::Unload_ToWaitPoint;
            }
        }
        else
        {
            bNavArrive_ = false;
            pRobotChassis_->SendGoal(iWaitPointID_);
            robot_state_ = RobotState::Unload_ToWaitPoint;
        }
    }
    return true;
}

bool Robot::Load_ArmToRangingPosture()
{
    if (bArmArrive_)
    {
        // 开启装货
        bArmArrive_ = false;
        pRobotArm_->ToPhotosPosture(2); // 移动到指定姿势，让测距传感器更加准确
        robot_state_ = RobotState::Load_ArmToPhotosPose2;
        qDebug() << "Load_ArmToPhotosPose2";

    }
    return true;
}

bool Robot::Load_LoadingDone_MoveBackward()
{
    if (bNavArrive_)
    {
        bNavArrive_ = false;
        bArmArrive_ = false;
        // lucien
//        pRobotArm_->ToRangingPosture();
//        robot_state_ = RobotState::Load_ArmToRangingPosture;    // 机械臂移动到测距姿态
        qDebug() <<"Load_ArmToRangingPosture";
    }
    return true;
}

}
