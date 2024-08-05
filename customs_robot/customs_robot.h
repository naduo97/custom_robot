#ifndef CUSTOMSROBOT_H
#define CUSTOMSROBOT_H
#include <thread>
#include <unordered_map>
#include <functional>
#include <QObject>
#include "task_management/task_management.h"
#include "robot_chassis/robot_chassis.h"
#include "robot_arm/robot_arm.h"
#include "visual_identity/visual_identity.h"
#include "LLog/LLog.h"
//test demo
//#include "test_demo/test_tcp_work.h"
//#include "test_demo/test_tcp_work2.h"

namespace CustomsRobot{

typedef std::function<bool(void)> WORK_FUN;
typedef std::function<void(const std::string&)> ERROR_CALLBACK;

enum RobotState
{
    Uninitialized = 0,      //  未初始化
    Ready = 1,              //  准备就绪

    Unload_ToMissionPoint = 2,     //  前往任务点
    Unload_ArmToPhotosPosture = 3, //  机械臂到拍照姿势
    Unload_ArmGrasping = 4,        //  机械臂抓取
    Unload_BoxIdentification = 5,  //  箱体识别
    Unload_ToPlatformPoint = 6,    //  前往月台点
    Unload_ToWaitPoint = 7,        //  前往等待点

    Unload_ToMissionPlatformPoint = 10, // 前往任务月台点
    Unload_ArmToPhotosPosture_Offset = 11,
    Unload_PlatformPointMove = 12,      // 月台点移动
    Unload_ManualMount = 13,            // 人工挂载
    Unload_ArmToRangingPosture = 14,    // 机械臂到测距姿势
    Unload_ArmToRangingPosture_1 = 15,
    Unload_ArmToRangingPosture_2 = 16,
    Unload_ArmToRangingPosture_3 = 17,
    Unload_ToWorkPoint = 18,            // 前往工作点
    Unload_ManualUnMount = 19,          // 人工卸载
    Unload_OpenCamera = 20,
    Upload_ArmToPhotosPosture = 1000,
//    Upload_ArmGrasping = 101,
    ArmUploading = 1002,       //  机械臂上货

    Pause = 8,              //  暂停
    Resume = 9,              //  恢复

    //装货流程 by JF Gan
    Load_Ready = 100,                   //  准备就绪（装货）
    Load_ToMissionPosition = 101,       //  前往任务点 （装货）
    Load_ArmToPhotosPose1 = 102,        //  机械臂拍照位姿1（计算车厢偏移距离）
    Load_ContainerOffset = 103,         //  计算当前车厢偏移距离
    Load_ArmOrigin = 104,               //  机械臂回零
    Load_ToOffsetPosition = 105,        //  移动车厢偏移距离到车厢中心线上
    Load_ManualStartLoad = 106,         //  人工启动装货
    Load_ToLoadPosition = 107,          //  移动到装货位置
    Load_ArmToPhotosPose2 = 108,     	//  机械臂拍照位姿2（计算货箱放置位姿）
    Load_ComputeBoxLoadPose = 109,      //  计算货箱放置位置
    Load_BoxLoading = 110,              //  放置货箱
    Load_ManualStopLoad = 111,          //  人工停止装货
    Load_ToPlatformPoint = 112,         //  前往装货月台点
    Load_ManualStopMission = 113,       //  人工结束本次任务
    Load_ToWaitPosition = 114,          //  前往等待点
    Load_ArmToRangingPosture = 115,     // 机械臂到测距姿势
    Load_LoadingDone_MoveBackward = 116 // 货箱装载完成 底盘往后移动
    // Load_Pause = 8,                  //  暂停
    // Load_Resume = 9                  //  恢复
};

class RoboStateDate
{
public:
    RoboStateDate()
    {
        bTaskStateUpdated_ = true;
        bChassisStateUpdated_ = true;
        bRobotArmStateUpdated_ = true;
        bVisualIdentityStateUpdated_ = true;
    }

    ~RoboStateDate(){}

    RoboStateDate& operator=(const TaskManagement::TaskStateData& data)
    {
        bTaskStateUpdated_ = true;
        this->TaskStateData_ = data;
        return *this;
    }

    RoboStateDate& operator=(const RobotChassis::ChassisStateData& data)
    {
        bChassisStateUpdated_ = true;
        this->ChassisStateData_ = data;
        return *this;
    }

    RoboStateDate& operator=(const RobotArm::RobotArmStateData& data)
    {
        bRobotArmStateUpdated_ = true;
        this->RobotArmStateData_ = data;
        return *this;
    }

    RoboStateDate& operator=(const VisualIdentity::VisualIdentityStateData& data)
    {
        bVisualIdentityStateUpdated_ = true;
        this->VisualIdentityStateData_ = data;
        return *this;
    }
    const RoboStateDate& GetObject()
    {
        return *this;
    }

public:
    bool bTaskStateUpdated_;
    TaskManagement::TaskStateData TaskStateData_;   // 任务模块的状态

    bool bChassisStateUpdated_;
    RobotChassis::ChassisStateData ChassisStateData_;   // 底盘模块的状态

    bool bRobotArmStateUpdated_;
    RobotArm::RobotArmStateData RobotArmStateData_; // 机械臂模块的状态

    bool bVisualIdentityStateUpdated_;
    VisualIdentity::VisualIdentityStateData VisualIdentityStateData_; // 视觉识别模块的状态

    RobotState robot_state_;
};

typedef std::function<bool(const RoboStateDate&)> UPDATE_CALLBACK;


class Robot : public QObject
{
    Q_OBJECT
public:
    Robot();
    /*  # updata_callback：更新数据的函数指针，用于向界面反馈当前模块的状态
        # error_callback：错误消息的函数指针，用于界面弹框或其他显示方法向用户提示*/
    Robot(UPDATE_CALLBACK updata_callback,\
          ERROR_CALLBACK error_callback);
    ~Robot();
    void RobotWork();   // 机器人的业务处理主要函数
    void SetUpdateCallBack(UPDATE_CALLBACK updata_callback);    // 设置更新数据的函数指针

    void TaskCallBack(const TaskManagement::TaskStateData& data);   // 任务模块的回调函数
    void ChassisStateCallBack(const RobotChassis::ChassisStateData& data);  // 底盘模块的回调函数
    void RobotArmStateCallBack(const RobotArm::RobotArmStateData& data);    // 机械臂模块的回调函数
    void VisualIdentityStateCallBack(const VisualIdentity::VisualIdentityStateData& data);  // 视觉识别模块的回调函数
    bool RobotPause();  //机器人暂停运行
    void RobotPauseStateBackup();    //
    bool RobotReset();  //机器人暂停运行
    bool RobotResumeToRun();  //机器人恢复运行
    bool CompleteMounting();    // 完成人工挂载
    bool CompleteUnMounting();    // 完成人工

    void SetConfirmVisualResultsMode(int mode_id);
private:
    bool Init();    // 初始化函数
    void Stop();    // 停止线程函数

private:    // 每个状态所对应的工作函数
    bool Uninitialized();        //  未初始化
    bool Ready();                //  准备就绪
    bool Unload_ToMissionPoint();       //  前往任务点
    bool Unload_ArmToPhotosPosture();   //  机械臂到拍照姿势
    bool Unload_ArmGrasping();          //  机械臂抓取
    bool ArmUploading();         //  机械臂装货
    bool Unload_BoxIdentification();    //  箱体识别
    bool Unload_ToPlatformPoint();      //  前往月台点
    bool Unload_ToWaitPoint();          //  前往等待点
    bool Unload_ToMissionPlatformPoint();   // 前往任务月台点
    bool Unload_ArmToPhotosPosture_Offset();
    bool Unload_PlatformPointMove();        // 月台点移动
    bool Unload_ManualMount();              // 人工挂载
    bool Unload_ArmToRangingPosture();      // 机械臂到测距姿势
    bool Unload_ArmToRangingPosture_1();      // 机械臂到测距姿势 1     采取三个点 的距离
    bool Unload_ArmToRangingPosture_2();      // 机械臂到测距姿势 2
    bool Unload_ArmToRangingPosture_3();      // 机械臂到测距姿势 3

    bool Unload_ToWorkPoint();              // 前往工作点
    bool Unload_ManualUnMount();            // 人工卸载
    bool Unload_OpenCamera();

    bool Pause();                //  暂停
    bool Resume();               //  恢复

    // 装货流程 by JF Gan
    bool Load_ToMissionPosition();       
    bool Load_ArmToPhotosPose1();     
    bool Load_ContainerOffset();     
    bool Load_ArmOrigin();           
    bool Load_ToOffsetPosition();    
    bool Load_ManualStartLoad();     
    bool Load_ToLoadPosition();      
    bool Load_ArmToPhotosPose2();    
    bool Load_ComputeBoxLoadPose();  
    bool Load_BoxLoading();          
    bool Load_ManualStopLoad();      
    bool Load_ToPlatformPoint();     
    bool Load_ManualStopMission();
    bool Load_ArmToRangingPosture();
    bool Load_LoadingDone_MoveBackward();

public:
    ERROR_CALLBACK pErrorCallback_;     // 错误弹框的回调函数指针
    UPDATE_CALLBACK pUpdateCallback;    // 数据更新的回调函数指针
    RobotState robot_state_;            // 记录当前机器人的工作状态
    RoboStateDate *pRobotStateData_;    // 记录机器人内每个模块的状态
    bool bIsStop_;                      // 是否停止
    std::thread thWork_;                // 工作线程类
    std::unordered_map<int, WORK_FUN> umapWorkFun;  // 哈希表，将每个状态和它所对应的函数绑定起来

public: // 相关模块的指针
    TaskManagement::TaskManagement *pTaskManagement_;
    RobotChassis::RobotChassis *pRobotChassis_;
    RobotArm::RobotArm *pRobotArm_;
    VisualIdentity::VisualIdentity *pVisualIdentity_;

private:    // 业务逻辑相关的状态变量
    LLOG::LLog log_;
    unsigned long long llWaitTime_;
    int iIdentificationRegion_;     // 记录当前拍照的位置，卸货目前需要一面拍两次
    Common::Pose WaitPoint_;        // 记录等待点的位置
    Common::Pose PlatformPoint_;    // 记录月台点位置
    int iWaitPointID_;
    int iPlatformPointID_;

    int iAdjustmentPointID_;        // 月台调整后的点ID
    bool bNavArrive_;               // 导航到达
    bool bArmArrive_;               // 机械臂执行到位
    bool bVisualFulfil_;            // 视觉检测完成
    bool bCompleteMount_;           // 人工挂载
    bool bCompleteUnMount_;         // 人工卸载
    bool bComputeMode_;             // 测量模式

    RobotState PauseBackup_RobotState_; // 暂停备份当前的 机器人状态
    bool bRobotPause;
    bool bRobotResume;              // 记录当前机器人需要恢复

    double dRangingEsult_[3];    // 只有三个点

    int iConfirmVisualResultsMode_; // 0:等待  1:确定  2:重拍

public:
    //装货流程 by JF Gan 
    bool bManualStartLoad_;         // 人工启动装货
    bool bManualStopLoad_;          // 人工结束装货 
    bool bManualStopMission_;        // 人工结束任务
    Common::Container CurrentContainer_;    // 当前集装箱信息
    Common::Box CurrentBox_;
    double LoadOffset = 100;        //装货时允许底盘误差前后10cm  
};
}
#endif // CUSTOMSROBOT_H
