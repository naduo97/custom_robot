#ifndef ROBOTARM_H
#define ROBOTARM_H
#include <QString>
#include <string>
#include <mutex>
#include <vector>
#include <functional>
#include <unordered_map>
#include <queue>
#include "work_base.h"
#include "common/common.h"
#include "zmcaux.h"
#include "zmotion.h"
#include "LLog/LLog.h"
#include "ranging_laser.h"
#include "ranging_laser_benewake.h"
#include "ranging_laser_long.h"
#include "component/axis_state.h"
#include "component/drive/laser_ranging/laser_ranging_jrt_bb2x.h"
namespace RobotArm{

enum RobotArmState
{
    Uninitialized = 0,  //  未初始化
    NetworkAnomaly = 1, //  通讯异常
    Normal = 2,         //  通讯正常
    ArmIsError = 3,     //  机械臂错误
    NotEnabled = 4,     //  未使能
    NoModelBuilt = 5    //  未建立模型
};

enum ExecuteState
{
    Idle = 0,       // 空闲
    Arrive = 1,     // 完成
    Stop = 2,       // 停止
    Pause = 3,      // 暂停
    Resume = 4,     // 恢复
    SinglePointMove = 5,    // 单点移动

    Unload_StateBegin = 100, //  卸货状态起点
    Unload_Arrive = 102,     //  完成
//    Unload_Fail = 103,       //  失败
    Unload_MoveingtoTarget = 104, //正在前往目标点
//    Unload_ArriveTarget = 105,    //到达目标点
//    Unload_MoveingtoTakePhotoPlace = 106, //正在前往拍照点
//    Unload_ArrivePhotoPlace = 107,          //到达拍照点
//    Unload_Grasping = 108,      //正在抓取
    Unload_GraspingDone = 109,  //抓取完成
    Unload_SingleGrasp = 110,   //单个抓取
//    Unload_Stop = 111,        // 停止
//    Unload_Pause = 112,       // 暂停
//    Unload_Resume = 113,      // 恢复
//    Unload_TcpHoming = 114,     // TCP（末端抓手） 归位
    Unload_AvrisCorrect_Suck = 115,  // 侧边箱体纠正 吸住箱体
    Unload_AvrisCorrect_Suck_up = 116,  // 侧边箱体纠正 吸住箱体 上提
    Unload_AvrisCorrect_Drag = 117,  // 侧边箱体纠正 拖动箱体
    Unload_AvrisCorrect_GoToGraspPoint = 118,    // 侧边箱体纠正 去到抓取点
    Unload_StateEndl = 199,       //  卸货状态结束点
// -------------------------------------------------
    Upload_StateBegin = 200,       //  上货状态起点
//    Upload_Underway = 201,   //  执行中
    Upload_Arrive = 202,     //  完成
//    Upload_Fail = 203,       //  失败
    Upload_MoveingtoTarget = 204, //正在前往目标点
//    Upload_ArriveTarget = 205,    //到达目标点
//    Upload_MoveingtoTakePhotoPlace = 206, //正在前往拍照点
//    Upload_ArrivePhotoPlace = 207,        //到达拍照点
//    Upload_Loading_singlefinish = 108,             //正在装货
    Upload_SingleLoadDone = 209,    //单个装货完成
    Upload_SingleLoading = 210,     //单个正在装货
    Upload_SingleLoading_RightSideCorrecting = 211,         //单个正在装货-右侧移动纠正
    Upload_SingleLoading_EjectionBox = 212,                 //单个正在装货-将货物抛出去
    Upload_SingleLoading_Correct_ToReadyPosition = 213,     //单个正在装货-移动到预备姿态
    Upload_SingleLoading_Correct_SwingLever = 214,          //单个正在装货-摆动前方用于拨正的杆子
    Upload_SingleLoading_Correct_BackOffCorrecting = 215,   //单个正在装货-后退纠正
    Upload_SingleLoading_Correct_ReturnPosition_MoveLeft = 216,      //单个正在装货-回到原上货位
    Upload_SingleLoading_Correct_ReturnPosition = 217,      //单个正在装货-回到原上货位
    Upload_SingleLoading_LengthwaysPush = 218,              //单个正在装货-纵向推压
    Upload_SingleLoading_MoveBackward = 219,                //单个正在装货-往后移动，避免拨杆与箱体干涉
    Upload_LoadingDone_MoveBackward = 220,                // 完成需要装载的货物，TCP完后移动

    //第二版小臂处理右侧箱子的逻辑
    Upload_SingleLoading_MovingLeft = 231,                  // 单个正在装货-移动到左侧
    Upload_SingleLoading_BaffleLaunching = 232,             // 单个正在装货-挡板推出
    Upload_SingleLoading_MovingRight = 233,                 // 单个正在装货-移动到右侧
    Upload_SingleLoading_BaffleStowed = 234,                // 单个正在装货-挡板收起
    Upload_SingleLoading_BackWarding = 235,                 // 单个正在装货-往后移动
    Upload_SingleLoading_ManualMove = 236,                  // 单个正在装货-人工移动
//    Upload_SingleRowLoadDone = 220,//单列装货完成
//    Upload_SingleSideloadDone = 221,  //单面装货完成
    Upload_StateEndl = 299,       //  上货状态结束点
    TestMode_TCPReciprocating_Count = 300,
    TestMode_TCPReciprocating_Time = 301,
    TestMode_EncircleMotion_MoveingToTarget = 302,
    TestMode_EncircleMotion_Arrive = 303
};

class RobotArmStateData
{
public:
    RobotArmStateData()
    {
        RobotArmState_ = RobotArmState::Uninitialized;
        ExecuteState_ = ExecuteState::Idle;
        iAxisGroupIndex_ = 1;
        iRobotStatus_ = 1;
        iRunMode_1_ = 1;
        iRunMode_2_ = 1;
        fL_1_ = 0.0;
        fL_2_ = 0.0;
        fL_3_ = 0.0;
        fL_4_ = 0.0;
        fL_5_ = 0.0;
        fL_6_ = 0.0;
        fTcp_x_ = 0.0;
        fTcp_y_ = 0.0;
        fTcp_z_ = 0.0;
        fTcp_rx_ = 0.0;
        fTcp_ry_ = 0.0;
        fTcp_rz_ = 0.0;
        dLeftLimitPoint_ = 0.0;
        dRightLimitPoint_ = 0.0;
        fLeftEdgeReturnAngle_ = 0.0;
        fLeftEdgeReturnDis_ = 0.0;
        fLeftEdgeReturnXDis_ = 0.0;
        fRightEdgeReturnAngle_ = 0.0;
        fRightEdgeReturnDis_ = 0.0 ;
        fRightEdgeReturnXDis_ = 0.0;
//        dLaserDistance_ = 0.0;
        iTestModeRunCount_ = 0;
        iTestModeEncircleMotionRunCount_ = 0;
    }
    ~RobotArmStateData(){}

public:
    RobotArmState RobotArmState_;
    ExecuteState ExecuteState_;


    int iAxisGroupIndex_;           //轴1到轴14的id 对应ui界面中的 AxisGroup checkedId
    QString sAxisGroupIndexStatus_;//选中轴的轴状态
    QString sAxisGroupIndexDpos_;  //选中轴的DPOS
    QString sAxisGroupIndexSpeed_; //选中轴的轴速度


    int iRobotStatus_;     //正逆解状态： 1为正解，2为逆解 默认1
    int iRunMode_1_;//单轴运动时 iRunMode_1_为持续运动方式，iRunMode_2_为点动方式
    int iRunMode_2_;

    std::unordered_map<int, int> umapAxisStatus_;   // 记录轴运动状态，0:运动中  -1:未运动
    std::unordered_map<int, std::string> umapAxisStatusString_;   // 记录轴运动状态，0:运动中  -1:未运动
    std::unordered_map<int, int> umapAxisMtype_;   // 记录轴运动状态，0:运动中  -1:未运动
    std::unordered_map<int, bool> umapIoOut_;   //记录IO口的输出(相对本软件)状态 例如：继电器控制
    std::unordered_map<int, int> umapIoInput_;  //记录IO口的输入(相对本软件)数值 例如：传感器

    QString sIn_9_Status_;
    QString sIn_10_Status_;
    QString sIn_11_Status_;
    QString sIn_12_Status_;
    QString sOut_0_status_;
    QString sOut_1_status_;

    float fL_1_;  //轴关节角
    float fL_2_;
    float fL_3_;
    float fL_4_;
    float fL_5_;
    float fL_6_;
    float fTcp_x_; //Tcp位姿
    float fTcp_y_;
    float fTcp_z_;
    float fTcp_rx_;
    float fTcp_ry_;
    float fTcp_rz_;
    float fT3_x_; //Tcp位姿
    float fT3_y_;
    float fT3_z_;
    float fT3_rx_;
    float fT3_ry_;
    float fT3_rz_;

//    double dLaserDistance_;
    double dLaserShortDist_;    // 短距离测距激光
    double dLaserLongDist_;     // 长距离测距激光
    double dLaserLeftDist_;     // 左侧测距激光
    double dLaserDownDist_;     // 下方测距激光

    double dLeftLimitPoint_;
    double dRightLimitPoint_;

    float fLeftEdgeReturnAngle_;
    float fLeftEdgeReturnDis_;
    float fLeftEdgeReturnXDis_;
    float fRightEdgeReturnAngle_;
    float fRightEdgeReturnDis_;
    float fRightEdgeReturnXDis_;

    int iTestModeRunCount_;
    int iTestModeEncircleMotionRunCount_;
};


class RobotArmControl
{
public:
    enum ControlType
    {
        Init = 0,           // 用于初始化
        AxisGroupIndex = 1, // 轴组合的下标ID发生改变
        InitIoOut = 2,      // 初始化IO口输出
        IoOut = 3,          // IO口输出
        InitIoInput = 4,    // 初始化IO口输入
        IoInput = 5,        // IO口输入
        RunButton = 6,      // 运行按键
        StopButton = 7,     // 停止按键
        SingleAxisReset = 8,    // 单轴的位置清零
        SingleAxisResetCoder = 9,    // 单轴的编码器清零
        AxisEnableSignal = 10,  // 轴电机使能
        StopAllAxis = 11,       // 停止全部轴
        TcpHoming = 12,         // 推杆归位
        TcpSingleRun = 13,      // tcp单独运动
        ResumeAllAxis = 16,       // 恢复全部轴
        RtRun = 17,
        RealRun = 18,
        Rapidstop = 19,
        TcpPush = 20,
        GetTcp = 21,
        SaveTcpAndT3 = 22,
        BtRt = 23,
        BtReal = 24,
        Tcp2Angle = 25,
        TEST1 = 26,
        TEST2 = 27,
        ConnectArm = 28,
        TestMode_StartRunCount = 29,
        TestMode_StartRunTime = 30,
        TestMode_StopRunCount = 31,
        TestMode_StopRunTime = 32,
        TestMode_StartRunConveyor = 33,
        TestMode_StopRunConveyor = 34,
        TestMode_StartEncircleMotion = 35,
        TestMode_StopEncircleMotion = 36
    };
public:
    RobotArmControl()
    {
        iAxisGroupIndex_ = 0;
        dUnits_ = 0.0;
        dSpeed_ = 0.0;
        dAccel_ = 0.0;
        dDecel_ = 0.0;
        dSramp_ = 0.0;
    }
    ~RobotArmControl(){}
public:
    std::string strData_;
    ControlType ControlType_;
    int iAxisGroupIndex_;
    int IoID_;
    bool IoState_;
    std::unordered_map<int, std::string> umapIoData_;
    std::vector<double> vecValue_;
    double dUnits_;
    double dSpeed_;
    double dAccel_;
    double dDecel_;
    double dSramp_;

    bool bMotorDirection_;
    int iRunModeGroupIndex_;
    double dMoveDistance_;
};

class PauseBackupData
{
public:
    PauseBackupData(){}
    ~PauseBackupData(){}
public:
    ExecuteState ExecuteState_;
    Common::Pose TargetPose_;
    Common::Box ExecutedBox_;
};


typedef std::function<void(const RobotArmStateData&)> STATE_CALLBACK;


class RobotArm : public CustomsRobot::WorkBase
{
public:
    RobotArm(STATE_CALLBACK call_back);
    ~RobotArm();

    void WorkFun();

public:
    STATE_CALLBACK pStateCallback_;
    LLOG::LLog log_;
public:
    void UpdateThread();
    void StopUpdateThread();
    void ComponentInit(); // 组件模块初始化
    void ControlData(const RobotArmControl& control_data);  // 来自界面的控制消息
    void IoOut(int id, bool state);
//    bool MoveAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float *pfDisancelist);
    bool MoveAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist
                 , float *pfDisancelist, float(&pdTargetAxisReal)[6]);
    bool IsEffectivePose(Common::Pose &pose);
    bool SetAxisSpeed(int* axis_list, int list_size, float speed, float accel, float decel);
    // 抓取箱体触发函数
    bool GraspingBox(const std::vector<Common::Pose> &box_pose, int pose_id);

    // 装货触发函数
//    bool UploadingBox(const std::vector<Common::Pose> &UpLoad_box_pose);
    bool UploadingBox(const Common::Container &container, const Common::Box &current_box);

    // 真空吸盘的工作函数
    void VacuumChuckFunction();

    //  移动到拍照姿势
    void ToPhotosPosture(int pose_id);
    //  移动到测距姿势
    void ToRangingPosture(int pose_id = 0);
    //  移动到折叠姿势
    void ToFoldingPosture();
    // 移动到指定位置
    void MoveArm(Common::Pose target_pose);

    void CameraControl(int mode_id);

    //界面点击运行按钮
    void RunSingleAxis(const RobotArmControl& control_data);
    //界面点击停止按钮
    void StopSingleAxis(const RobotArmControl& control_data);

    void SetLimitPoint_(int type, double dValue);
    void SingleAxisReset(const RobotArmControl& control_data);
    void SingleAxisResetCoder(const RobotArmControl& control_data);

    void StopAllAxis();

    void Pause();
    void PauseBackup();
    void ResumeToRun();
    void Reset();

    void TcpHoming();   // 吸盘架归零
    void TcpPush();     // 推出吸盘架
    void tcp_single_run();  // 吸盘架 单独运动
    void tcp_single_run(float start_pos); // 吸盘架 单独运动

    void GetTcpData();

    void SetLeftEdgeReturnAngle(float fValue);
    void SetLeftEdgeReturnDis(float fValue);
    void SetLeftEdgeReturnXDis(float fValue);
    void SetRightEdgeReturnAngle(float fValue);
    void SetRightEdgeReturnDis(float fValue);
    void SetRightEdgeReturnXDis(float fValue);

//    double GetPointLaserDistance();
//    double GetPointLaserShortDistance();
    double GetPointLaserLongDistance();
    double GetPointLaserLeftDistance();
    double GetPointLaserDownDistance();

    bool isArriveThisPlace(float array[6]);


    bool ForearmIsExistBox();       // 判断前臂是不是存在货物
    bool BigArmBeltRun(float fSpeed, int iDir);   // 大臂传送带 运动
    bool BigArmBeltStop();          // 大臂传送带 停止
//    void DirectToInverseFrameTans(float* pfDirectPose, float fAngle, float* pfInversePose);

    void DirectToInverseFrameTans(float* direct_pose, float* inverse_pose);
    void InverseToDirectFrameTans(float* inverse_pose, float* direct_pose);

    void ComputeTcpPose(float* temp_pose, int position_num, float distance,
                                   int rotation_num, float angle, float* target_pose);

    Eigen::Matrix4d PoseToMatrix4d(std::vector<float> &xyzrpy);
    int GetRefinedPose(Common::Pose &box_pose, double y_l, double y_r);

    bool GetPhotoSensorIsTriggered(int ID);    // 获取光电传感器的状态 ID号  -1：没有该设备
public:

    bool bRepeatFunction_;
    Common::Pose TargetPose_;
    float TargetAxisReal_[6];     // 目标正解轴角度
    Common::Box ExecutedBox_; // 当前执行的货物
    int ExecutedBoxLabel_ = -1;
    std::queue<Common::Box> qLoadBox_;  // 待装载货物队列
    std::queue<Common::Box> qUnloadBox_;
    std::queue<Common::Pose> qEncircleMotionPose_;  // 环绕运动 位置队列
//    std::queue<Common::Pose> qBoxPose_;
//    std::queue<int> qBoxPoseLabel_;
    long long llSendTime_;
    RobotArmStateData RobotArmStateData_;
    RobotArmStateData RobotArmStateOldData_;    // 记录暂停前的状态

    RobotArmStateData PauseBackupStateData_;    // 暂停备份状态

    PauseBackupData PauseBackupData_;

    bool bRobotArmResume_;
    bool bManualMoveDone_;
    float fDpos_13_;//推杆位置
    int axis_real_list[6] = {0,1,2,3,4,5}; //轴列表
    int axis_rt_list[6] = {6,7,8,9,10,11};
    int left_or_right;//装货时 下一列的方向  1为左 2为右
    int iMode_14_; //装货时 轴14运动控制标识符
    int iMode_12_; //卸货时 轴12运动控制标识符
    int iTuboMode_;//卸货时 吸气和吹气控制标识符
    ZMC_HANDLE ZMC_Handle_; //连接标识符
    char ip_[16];
    bool bTryGrasp_;
    bool bDepthOffset_;
    long long llGraspWaitTime_;

    double box_height = 200.0; //箱子高度200mm
    double box_length = 200.0; //箱子长度200mm
    double box_width = 400.0;  //箱子长度200mm
    double first_z;
    bool bLoadingDone_ = false;
    bool bRobotArmPause_;

//    RangingLaser LaserShortDist_;   // 短距离测距激光
//    RangingLaser_Long LaserLongDist_;    // 长距离测距激光
//    RangingLaser_Benewake LaserDownDist_;
    LaserRangingJrtBb2x FrontLaserDist_;    // 向前的激光测距

    AxisState ForearmConveyorBelt_; // 大臂传送带
    AxisState BigarmConveyorBelt_;  // 小臂传送带
    AxisState BackConveyorBelt_;    // 背滚传送带
    AxisState BigarmSuctionCupBracket_; // 小臂吸盘架

    bool bIsStopUpdate_;
    std::thread thUpdateThread_;

    int iTestModeRunCount_;
    long long llTestModeRunTime_;
    bool bTestModeEnablementConveyor_;
    bool bIsPushedOut_;
    long long llWaitingTime_;
    std::mutex mtxControl_;
};
}
#endif // ROBOTARM_H
