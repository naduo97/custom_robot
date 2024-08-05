#include "dialog.h"
#include "ui_dialog.h"
#include <iostream>
#include <QMessageBox>
#include <QSizePolicy>
#include <QTextCodec>
#include <QDebug>
#include <string>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <bitset>
#include "common/image_check_box.h"

#define CONFIG_PATH "../config/"

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
    , pCustomsRobot_(nullptr)
{
    ui->setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint);
//    this->setGeometry(0,0,900,500);
//    this->setGeometry(0,0,1624,768);

    pParamCfgPage_ = nullptr;

    bInit_ = false;
    pMainPage_ = nullptr;
    ImageCheckBox *imagebox = new ImageCheckBox(this);
//    imagebox->setGeometry(0,0,40,40);
    imagebox->setMaximumSize(40,40);
    imagebox->setMinimumSize(40,40);
    QList<QIcon> icons;
    icons << QIcon("./config/icon/Panic_Button_1.png")<<QIcon("./config/icon/Panic_Button_2.png")
           <<QIcon("./config/icon/Panic_Button_4.png")<<QIcon("./config/icon/Panic_Button_4.png");
    imagebox->setText("");
    imagebox->setVisible(true);
    imagebox->setImage(icons);

    ui->horizontalLayout_2->addWidget(imagebox);
//    strRobotParaFileName_ = "robot_para.ini";
//    QSettings configIni(strRobotParaFileName_, QSettings::IniFormat);
//    if (!configIni.contains(QString("config/versions")))
//    {
//        QMessageBox::critical(this, "ERROR", "without robot parameters file!");
//        exit(1);
//    }
    pCustomsRobot_ = new CustomsRobot::Robot(std::bind(&Dialog::ShowData, this, std::placeholders::_1), \
                                             std::bind(&Dialog::ErrorCallback, this, std::placeholders::_1));
    if (pCustomsRobot_->pVisualIdentity_->bStartLive_ == true)
    {
        pCustomsRobot_->pVisualIdentity_->StartLive(std::bind(&Dialog::LiveStreamCallBack, this, std::placeholders::_1));
    }
    Init();
    bInit_ = true;
}

Dialog::~Dialog()
{
    delete pCustomsRobot_;
    if (pMainPage_)
    {
        delete pMainPage_;
    }
    if (pTaskManagementPage_)
    {
        delete pTaskManagementPage_;
    }
    if (pRobotChassisPage_)
    {
        delete pRobotChassisPage_;
    }
    if(pRobotArmPage_) {
        delete pRobotArmPage_;
    }
    if (pVisualIdentityPage_)
    {
        delete pVisualIdentityPage_;
    }
    delete ui;
//    if (imagebox != nullptr)
//    {
//        delete imagebox;
//    }
}

void Dialog::LoadININ()
{
    strRobotParaFileName_ = "robot_para.ini";
    QSettings tempConfigIni(CONFIG_PATH + strRobotParaFileName_, QSettings::IniFormat);
//    tempConfigIni.setIniCodec(QTextCodec::codecForName("utf-8"));
    if (!tempConfigIni.contains(QString("config/versions")))
    {
        QMessageBox::critical(this, "ERROR", "without robot parameters file!");
//        exit(1);
    }
//    tempConfigIni.value(QString(""), 1).toInt();
    // 机械臂
    if (pRobotArmPage_)
    {
        QString strRobotArmParaFileName = "robot_arm_para.ini";
        QSettings tempArmConfigIni(CONFIG_PATH + strRobotArmParaFileName, QSettings::IniFormat);
//        tempArmConfigIni.setIniCodec(QTextCodec::codecForName("utf-8"));
        if (!tempArmConfigIni.contains(QString("config/versions")))
        {
            QMessageBox::critical(this, "ERROR", "without robot arm parameters file!");
//            exit(1);
        }
        pRobotArmPage_->LoadConfig(tempArmConfigIni);
    }
    // 任务
    if (pTaskManagementPage_)
    {
        pTaskManagementPage_->LoadConfig(tempConfigIni);
    }
    // 视觉
    if (pVisualIdentityPage_)
    {
        QString strVisualParaFileName_1 = "box_poses_1.ini";
        QSettings tempVisualConfigIni_1(CONFIG_PATH + strVisualParaFileName_1, QSettings::IniFormat);
//        tempVisualConfigIni_1.setIniCodec(QTextCodec::codecForName("utf-8"));
        if (!tempVisualConfigIni_1.contains(QString("config/versions")))
        {
            QMessageBox::critical(this, "ERROR", "without box_poses_1.ini file!");
//            exit(1);
        }
        pVisualIdentityPage_->LoadConfig(tempVisualConfigIni_1);

        QString strVisualParaFileName_2 = "box_poses_2.ini";
        QSettings tempVisualConfigIni_2(CONFIG_PATH + strVisualParaFileName_2, QSettings::IniFormat);
//        tempVisualConfigIni_2.setIniCodec(QTextCodec::codecForName("utf-8"));
        if (!tempVisualConfigIni_2.contains(QString("config/versions")))
        {
            QMessageBox::critical(this, "ERROR", "without box_poses_2.ini file!");
//            exit(1);
        }
        pVisualIdentityPage_->LoadConfig(tempVisualConfigIni_2);

        //5255.0, 750.0, 1160.0, 0.0, 0.0, 0.0 左上角
//        QString strVisualIdentityFileName = "box_poses_1.ini";
//        QSettings tempArmConfigIni(strVisualIdentityFileName, QSettings::IniFormat);
//        tempArmConfigIni.setIniCodec(QTextCodec::codecForName("utf-8"));

//        int temp_num = 0;
//        for (int row = 1; row < 7; ++row)
//        {
//            if (row % 2 == 0)
//            {
//                for (int column = 4; column >= 0; --column)
//                {
//                    tempArmConfigIni.beginGroup(QString("box") + QString::number(temp_num));
//                    tempArmConfigIni.setValue(QString("x"), 5255.0);
//                    tempArmConfigIni.setValue(QString("y"), 750.0 - (column * 38 * 10));
//                    tempArmConfigIni.setValue(QString("z"), 80.0 - ((row - 1) * 18 * 10));
//                    tempArmConfigIni.setValue(QString("pitch"), 0.0);
//                    tempArmConfigIni.setValue(QString("roll"), 0.0);
//                    tempArmConfigIni.setValue(QString("yaw"), 0.0);
//                    tempArmConfigIni.endGroup();
//                    temp_num++;
//                }
//            }
//            else
//            {
//                for (int column = 0; column < 5; ++column)
//                {
//                    tempArmConfigIni.beginGroup(QString("box") + QString::number(temp_num));
//                    tempArmConfigIni.setValue(QString("x"), 5255.0);
//                    tempArmConfigIni.setValue(QString("y"), 750.0 - (column * 38 * 10));
//                    tempArmConfigIni.setValue(QString("z"), 80.0 - ((row - 1) * 18 * 10));
//                    tempArmConfigIni.setValue(QString("pitch"), 0.0);
//                    tempArmConfigIni.setValue(QString("roll"), 0.0);
//                    tempArmConfigIni.setValue(QString("yaw"), 0.0);
//                    tempArmConfigIni.endGroup();
//                    temp_num++;
//                }
//            }

//        }
    }

    // 底盘
    if (pRobotChassisPage_)
    {
        QString strChassisParaFileName = "chassis_para.ini";
        QSettings tempChassisConfigIni(CONFIG_PATH + strChassisParaFileName, QSettings::IniFormat);
//        tempChassisConfigIni.setIniCodec(QTextCodec::codecForName("utf-8"));
        if (!tempChassisConfigIni.contains(QString("config/versions")))
        {
            QMessageBox::critical(this, "ERROR", "without chassis_para.ini file!");
//            exit(1);
        }
        pRobotChassisPage_->LoadConfig(tempChassisConfigIni);
    }
}

void Dialog::Init()
{
    QSettings configIni;
    // -----------初始化页面--------------
    pMainPage_ = new MainPage;
    umapPage_[u8"主界面"] = ui->stackedWidget->addWidget(pMainPage_);
    connect(pMainPage_, &MainPage::LiveTypeChange, this, &Dialog::LiveStreamTypeChange);

    pTaskManagementPage_ = new TaskManagementPage;
    connect(pTaskManagementPage_, &TaskManagementPage::SendTaskData, this, &Dialog::TaskControlData);
    umapPage_[u8"任务"] = ui->stackedWidget->addWidget(pTaskManagementPage_);

    pRobotChassisPage_ = new RobotChassisPage;
    connect(pRobotChassisPage_, &RobotChassisPage::SendChassisControl, this, &Dialog::ChassisControlData);
    umapPage_[u8"底盘"] = ui->stackedWidget->addWidget(pRobotChassisPage_);

    pRobotArmPage_ = new RobotArmPage;
    connect(pRobotArmPage_, &RobotArmPage::SendRobotArmControl, this, &Dialog::RobotArmControlData);
    umapPage_[u8"机械臂"] = ui->stackedWidget->addWidget(pRobotArmPage_);

    pVisualIdentityPage_ = new VisualIdentityPage;
    connect(pVisualIdentityPage_, &VisualIdentityPage::ModifyingArtificialData, this, &Dialog::VisualIdentityEnabledArtificialData);
    connect(pVisualIdentityPage_, &VisualIdentityPage::SendVisualIdentityControl, this, &Dialog::VisualIdentityControlData);
    umapPage_[u8"视觉"] = ui->stackedWidget->addWidget(pVisualIdentityPage_);

    pParamCfgPage_ = new ParamCfgPage;
    umapPage_[u8"参数设置"] = ui->stackedWidget->addWidget(pParamCfgPage_);
    connect(pParamCfgPage_, &ParamCfgPage::SetParameter, this, &Dialog::ParameterSetControlData);

    ui->stackedWidget->setCurrentIndex(2);
    // -----------初始化页面选择--------------
    QListWidgetItem* pItem0 = new QListWidgetItem(ui->listWidget);
    pItem0->setText(u8"主界面");
    pItem0->setTextAlignment(Qt::AlignHCenter);
    pItem0->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEnabled);

    QListWidgetItem* pItem1 = new QListWidgetItem(ui->listWidget);
    pItem1->setText(u8"任务");
    pItem1->setTextAlignment(Qt::AlignHCenter);
    pItem1->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEnabled);

    QListWidgetItem* pItem2 = new QListWidgetItem(ui->listWidget);
    pItem2->setText(u8"机械臂");
    pItem2->setTextAlignment(Qt::AlignHCenter);
    pItem2->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEnabled);

    QListWidgetItem* pItem3 = new QListWidgetItem(ui->listWidget);
    pItem3->setText(u8"底盘");
    pItem3->setTextAlignment(Qt::AlignHCenter);
    pItem3->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEnabled);

    QListWidgetItem* pItem4 = new QListWidgetItem(ui->listWidget);
    pItem4->setText(u8"视觉");
    pItem4->setTextAlignment(Qt::AlignHCenter);
    pItem4->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEnabled);

    QListWidgetItem* pItem5 = new QListWidgetItem(ui->listWidget);
    pItem5->setText(u8"参数设置");
    pItem5->setTextAlignment(Qt::AlignHCenter);
    pItem5->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEnabled);

    LoadININ();
    ui->listWidget->setCurrentRow(0);
}

// 界面显示刷新
bool Dialog::ShowData(const CustomsRobot::RoboStateDate& state_data)
{
    if (!bInit_)
    {
        return false;
    }
    // 任务界面
    if (pTaskManagementPage_ && state_data.bTaskStateUpdated_ && ui->stackedWidget->currentIndex() == 3)
    {
        pTaskManagementPage_->Update(state_data.TaskStateData_);
    }
    // 底盘界面
    if (pRobotChassisPage_ && state_data.bChassisStateUpdated_ && ui->stackedWidget->currentIndex() == 4)
    {
        pRobotChassisPage_->Update(state_data.ChassisStateData_);
    }
    // 视觉识别
    if (pVisualIdentityPage_ && state_data.bVisualIdentityStateUpdated_ ) // && ui->stackedWidget->currentIndex() == 6
    {
        pVisualIdentityPage_->Update(state_data.VisualIdentityStateData_);
    }
    // 机械臂
    if (pRobotArmPage_ && state_data.bRobotArmStateUpdated_  && ui->stackedWidget->currentIndex() == 5)
    {
        pRobotArmPage_->Update(state_data.RobotArmStateData_);
    }

    if (pParamCfgPage_ && (state_data.bVisualIdentityStateUpdated_ || state_data.bRobotArmStateUpdated_)  && ui->stackedWidget->currentIndex() == 7)
    {
        pParamCfgPage_->Update(state_data.RobotArmStateData_, state_data.VisualIdentityStateData_);
    }

    if (state_data.robot_state_ == CustomsRobot::RobotState::Pause)
    {
        if (ui->PauseOrResumeButton->text() != u8"恢复继续")
        {
            ui->PauseOrResumeButton->setText(u8"恢复继续");
        }
        ui->bt_Reset->setVisible(true);
    }
    else
    {
        if (ui->PauseOrResumeButton->text() != u8"暂停")
        {
            ui->PauseOrResumeButton->setText(u8"暂停");
        }
        ui->bt_Reset->setVisible(false);
    }

//    if (state_data.RobotArmStateData_.ExecuteState_ == RobotArm::ExecuteState::Pause)
//    {
//        ui->bt_Resume->setVisible(true);
//    }
//    else
//    {
//        ui->bt_Resume->setVisible(false);
//    }
    return true;
}

void Dialog::ErrorCallback(const std::string &error_data)
{
    QMessageBox::critical(this, "ERROR", error_data.c_str());
}

// 视频流回调函数
void Dialog::LiveStreamCallBack(const cv::Mat& mat_data)
{
    if (ui->stackedWidget->currentIndex() == 2)
    {
        pMainPage_->UpdateLiveStream(mat_data);
    }
    if (pParamCfgPage_ && ui->stackedWidget->currentIndex() == 7)
    {
        pParamCfgPage_->UpdateLiveStream(mat_data);
    }

}

// 机械臂控制消息
void Dialog::RobotArmControlData(const RobotArm::RobotArmControl& control_data)
{
    if (pCustomsRobot_)
    {
        if (pCustomsRobot_->pRobotArm_)
        {
            pCustomsRobot_->pRobotArm_->ControlData(control_data);
            // RobotArm::RobotArmControl::ControlType::AxisEnableSignal
            if (control_data.ControlType_ == RobotArm::RobotArmControl::ControlType::AxisEnableSignal)
            {
//                if (pCustomsRobot_->robot_state_ > CustomsRobot::RobotState::Ready)
//                {
//                    pCustomsRobot_->robot_state_ = CustomsRobot::RobotState::Ready;
//                }
            }
        }
    }
}

// 视觉模块控制消息
void Dialog::VisualIdentityControlData(const VisualIdentity::VisualIdentityControl& control_data)
{
    if (pCustomsRobot_)
    {
        if (pCustomsRobot_->pVisualIdentity_)
        {
            if (control_data.ControlType_ == VisualIdentity::VisualIdentityControl::BoxDetection)
            {
                pCustomsRobot_->pVisualIdentity_->ActivationIdentification(control_data.iDataId_);
            }
            else if (control_data.ControlType_ == VisualIdentity::VisualIdentityControl::ConfirmVisualResults)
            {
//                pCustomsRobot_->pVisualIdentity_->ActivationIdentification(control_data.iDataId_);
                pCustomsRobot_->SetConfirmVisualResultsMode(1);

            }
            else if (control_data.ControlType_ == VisualIdentity::VisualIdentityControl::RetakeRecognition)
            {
//                pCustomsRobot_->pVisualIdentity_->ActivationIdentification(control_data.iDataId_);
                pCustomsRobot_->SetConfirmVisualResultsMode(2);
            }
            else
            {
                pCustomsRobot_->pVisualIdentity_->ControlData(control_data);
            }
        }
    }
}

void Dialog::ChassisControlData(const RobotChassis::ChassisControl& control_data)
{
    if (pCustomsRobot_)
    {
        if (pCustomsRobot_->pRobotChassis_)
        {
            pCustomsRobot_->pRobotChassis_->ControlData(control_data);
        }
    }
}

// 任务界面控制
void Dialog::TaskControlData(const TaskManagement::TaskControl &data)
{
    if (pCustomsRobot_)
    {
        if (data.ControlType_ == TaskManagement::TaskControl::ControlType::PublishTask)
        {
            if (pCustomsRobot_->pTaskManagement_)
            {
                pCustomsRobot_->pTaskManagement_->AddTask(data.TaskData_);
            }
        }
        else if (data.ControlType_ == TaskManagement::TaskControl::ControlType::CompleteMounting)
        {   // 确认人工完成挂载
            if (pCustomsRobot_->pRobotArm_)
            {
                if (pCustomsRobot_->pRobotArm_->RobotArmStateData_.RobotArmState_ < RobotArm::RobotArmState::Normal)
                {
                    qDebug() << "机械臂未连接";
                }
                else
                {
                    pCustomsRobot_->CompleteMounting();
                }
            }
        }
        else if (data.ControlType_ == TaskManagement::TaskControl::ControlType::CompleteUnMounting)
        {   // 确认人工完成卸载
            pCustomsRobot_->CompleteUnMounting();
        }
        else if (data.ControlType_ == TaskManagement::TaskControl::ControlType::CompleteLoading)
        {   // 完成装货按钮
            // 更新当前 状态
            if (pCustomsRobot_->robot_state_ == CustomsRobot::RobotState::Load_BoxLoading)
            {
                pCustomsRobot_-> bManualStopLoad_ = true;
                pCustomsRobot_->robot_state_ = CustomsRobot::RobotState::Load_ManualStopLoad;
            }
        }
    }
}


void Dialog::VisualIdentityEnabledArtificialData(bool data)
{
    if (pCustomsRobot_)
    {
        if (pCustomsRobot_->pVisualIdentity_)
        {
            pCustomsRobot_->pVisualIdentity_->EnabledArtificialData(data);
        }
    }
}

void Dialog::on_listWidget_itemClicked(QListWidgetItem *item)
{
    ui->stackedWidget->setCurrentIndex(umapPage_[item->text().toStdString()]);
    switch (ui->stackedWidget->currentIndex()) {
    case 2: // 主界面
    {
        break;
    }
    case 3:  // 任务
    {
        if (pTaskManagementPage_)
        {
            pTaskManagementPage_->Update(pCustomsRobot_->pRobotStateData_->TaskStateData_);
        }
        break;
    }
    case 4:  // 底盘
    {
        if (pRobotChassisPage_)
        {
            pRobotChassisPage_->Update(pCustomsRobot_->pRobotStateData_->ChassisStateData_);
        }
        break;
    }
    case 5:  // 机械臂
    {
        if (pRobotArmPage_)
        {
            pRobotArmPage_->Update(pCustomsRobot_->pRobotStateData_->RobotArmStateData_);
        }
        break;
    }
    case 6:  // 视觉
    {
        if (pVisualIdentityPage_)
        {
            pVisualIdentityPage_->Update(pCustomsRobot_->pRobotStateData_->VisualIdentityStateData_);
        }
        break;
    }
    case 7:  // 参数设置
    {
        if (pParamCfgPage_)
        {
            pParamCfgPage_->Update(pCustomsRobot_->pRobotStateData_->RobotArmStateData_
                            , pCustomsRobot_->pRobotStateData_->VisualIdentityStateData_);
        }
        break;
    }
    default:
        break;
    }
}

void Dialog::LiveStreamTypeChange(const int& type)
{
    pCustomsRobot_->pVisualIdentity_->ChangeLiveType(type);
}

void Dialog::ParameterSetControlData(const ParameterConfigControl &data)
{
//    qDebug() << "ParameterSetControlData -- Control Type : " << data.ControlType_;
    switch (data.ControlType_) {
    case ParameterConfigControl::ControlType::SetTopROI:
    {
        if (pCustomsRobot_->pVisualIdentity_)
        {
            pCustomsRobot_->pVisualIdentity_->SetROI(1, data.Roi_);
        }
        break;
    }
    case ParameterConfigControl::ControlType::SetBottomROI:
    {
        if (pCustomsRobot_->pVisualIdentity_)
        {
            pCustomsRobot_->pVisualIdentity_->SetROI(2, data.Roi_);
        }
        break;
    }
    case ParameterConfigControl::ControlType::QuLimit:
    {
        if (pCustomsRobot_->pVisualIdentity_)
        {
            pCustomsRobot_->pVisualIdentity_->UV2RealPoint(data.vecValue_[0], data.vecValue_[1]);
        }
        break;
    }
    case ParameterConfigControl::ControlType::SetLeftLimit:
    {
        if (pCustomsRobot_->pRobotArm_)
        {
            pCustomsRobot_->pRobotArm_->SetLimitPoint_(0, data.vecValue_[0]);
        }
        break;
    }
    case ParameterConfigControl::ControlType::SetRightLimit:
    {
        if (pCustomsRobot_->pRobotArm_)
        {
            pCustomsRobot_->pRobotArm_->SetLimitPoint_(1, data.vecValue_[0]);
        }
        break;
    }
    case ParameterConfigControl::ControlType::MoveArmToTop:
    {
        if (pCustomsRobot_->pRobotArm_)
        {
            pCustomsRobot_->pRobotArm_->ToPhotosPosture(1);
        }
        break;
    }
    case ParameterConfigControl::ControlType::MoveArmToBottom:
    {
        if (pCustomsRobot_->pRobotArm_)
        {
            pCustomsRobot_->pRobotArm_->ToPhotosPosture(2);
        }
        break;
    }
    case ParameterConfigControl::ControlType::MoveArmToLimitDetection:
    {
        if (pCustomsRobot_->pRobotArm_)
        {
            pCustomsRobot_->pRobotArm_->MoveArm(Common::Pose(5460, 0, 1000, 0, 0, 0));
        }
        break;
    }
    case ParameterConfigControl::ControlType::OpenCamera:
    {
        if (pCustomsRobot_->pRobotArm_)
        {
            pCustomsRobot_->pRobotArm_->CameraControl(1);
        }
        break;
    }
    case ParameterConfigControl::ControlType::CloseCamera:
    {
        if (pCustomsRobot_->pRobotArm_)
        {
            pCustomsRobot_->pRobotArm_->CameraControl(0);
        }
        break;
    }
    case ParameterConfigControl::ControlType::SetScoreValue:
    {
        if (pCustomsRobot_->pVisualIdentity_)
        {
            pCustomsRobot_->pVisualIdentity_->SetScoreValue(data.fTempData_);
        }
        break;
    }
    case ParameterConfigControl::ControlType::LeftEdgeReturnAngle:
    {
        if (pCustomsRobot_->pRobotArm_)
        {
//            pCustomsRobot_->pRobotArm_->SetScoreValue(data.fTempData_);
        }
        break;
    }
    case ParameterConfigControl::ControlType::LeftEdgeReturnDis:
    {
//        if (pCustomsRobot_->pVisualIdentity_)
//        {
//            pCustomsRobot_->pVisualIdentity_->SetScoreValue(data.fTempData_);
//        }
        break;
    }
    case ParameterConfigControl::ControlType::LeftEdgeReturnXDis:
    {
//        if (pCustomsRobot_->pVisualIdentity_)
//        {
//            pCustomsRobot_->pVisualIdentity_->SetScoreValue(data.fTempData_);
//        }
        break;
    }
    default:
        break;
    }
}

void Dialog::on_PauseOrResumeButton_clicked()
{
    if (ui->PauseOrResumeButton->text() == u8"暂停")
    {
        // 机器人向模块发出暂停指令
        pCustomsRobot_->RobotPause();
    }
    else
    {
        // 机器人向模块发出恢复指令
        pCustomsRobot_->RobotResumeToRun();
    }
}

// 复位
void Dialog::on_bt_Reset_clicked()
{
    pCustomsRobot_->RobotReset();
}

void Dialog::on_bt_Exit_clicked()
{
//    exit(1);
    this->close();
}
