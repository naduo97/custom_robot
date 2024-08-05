#include "robot_chassis_page.h"
#include "ui_robot_chassis_page.h"
#include <QRegularExpressionValidator>

RobotChassisPage::RobotChassisPage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RobotChassisPage)
{
    ui->setupUi(this);
    ui->angleEdit->setValidator(new QRegularExpressionValidator(QRegularExpression("^-?(?:[0-8]\\d{0,3}|9000)(?:\\.\\d+)?$")));

    ui->lidarImage->clear();
    QPalette palette;
    palette.setColor(QPalette::Window, QColor(200, 200, 200));
    ui->lidarImage->setAutoFillBackground(true);
    ui->lidarImage->setPalette(palette);
    ui->lidarImage->setMinimumSize(220, 220);
}

RobotChassisPage::~RobotChassisPage()
{
    delete ui;
}

void RobotChassisPage::Update(const RobotChassis::ChassisStateData &data)
{
    ui->CurrentXEdit->setText(QString::number(data.Pose_.x_));
    ui->CurrentYEdit->setText(QString::number(data.Pose_.y_));
    ui->CurrentZEdit->setText(QString::number(data.Pose_.z_));
    ui->CurrentOXEdit->setText(QString::number(data.Pose_.qx_));
    ui->CurrentOYEdit->setText(QString::number(data.Pose_.qy_));
    ui->CurrentOZEdit->setText(QString::number(data.Pose_.qz_));
    ui->CurrentOWEdit->setText(QString::number(data.Pose_.qw_));

    ui->TargetXEdit->setText(QString::number(data.TargetPose_.x_));
    ui->TargetYEdit->setText(QString::number(data.TargetPose_.y_));
    ui->TargetZEdit->setText(QString::number(data.TargetPose_.z_));
    ui->TargetOXEdit->setText(QString::number(data.TargetPose_.qx_));
    ui->TargetOYEdit->setText(QString::number(data.TargetPose_.qy_));
    ui->TargetOZEdit->setText(QString::number(data.TargetPose_.qz_));
    ui->TargetOWEdit->setText(QString::number(data.TargetPose_.qw_));

    QString strChassisState;
    switch(data.ChassisState_)
    {
        case RobotChassis::ChassisState::Uninitialized :  //  未初始化
        {
            strChassisState = "未初始化";
            break;
        }
        case RobotChassis::ChassisState::NetworkNormal : //  通讯正常
        {
            strChassisState = "通讯正常";
            break;
        }
        case RobotChassis::ChassisState::NetworkAnomaly : //  通讯异常
        {
            strChassisState = "通讯异常";
            break;
        }
        case RobotChassis::ChassisState::ChassisError : //  底盘错误
        {
            strChassisState = "底盘错误";
            break;
        }
        default :
        {
            strChassisState = "未知";
            break;
        }
    }
    ui->ChassisStateEdit->setText(strChassisState);
    QString strNavState;
    switch(data.NavigationState_)
    {
        case RobotChassis::NavigationState::Idie :  //  空闲
        {
            strNavState = "空闲";
            break;
        }
        case RobotChassis::NavigationState::Underway : //  进行中
        {
            strNavState = "进行中";
            break;
        }
        case RobotChassis::NavigationState::Arrive : //  完成
        {
            strNavState = "完成";
            break;
        }
        case RobotChassis::NavigationState::Pause : //  完成
        {
            strNavState = "暂停中";
            break;
        }
        default :
        {
            strNavState = "未知";
            break;
        }
    }
    ui->NavStateEdit->setText(strNavState);

    ui->voltage_label->setText(QString::number(data.dVoltage_) + "%");
    // 激光雷达图
    QImage neworiginalImage = QImage((const unsigned char*)(data.matLidarImage_.data), \
                             data.matLidarImage_.cols, data.matLidarImage_.rows, \
                             data.matLidarImage_.step, QImage::Format_RGB888);
    if (!neworiginalImage.isNull())
    {
        ui->lidarImage->setPixmap(QPixmap::fromImage(neworiginalImage.scaled(ui->lidarImage->width(), ui->lidarImage->height(),Qt::KeepAspectRatio, Qt::FastTransformation)));
    }
}

void RobotChassisPage::LoadConfig(QSettings &configIni)
{
    RobotChassis::ChassisControl StationsInitData;
    StationsInitData.ControlType_ = RobotChassis::ChassisControl::ControlType::Init;
    StationsInitData.strValue_ = configIni.value(QString("config/StationsFile")).toString().toStdString();
    emit SendChassisControl(StationsInitData);

    RobotChassis::ChassisControl NMCInitData;
    NMCInitData.ControlType_ = RobotChassis::ChassisControl::ControlType::NMC_Init;
    NMCInitData.strValue_ = configIni.value(QString("config/nmc_ip")).toString().toStdString();
    NMCInitData.iValue_ = configIni.value(QString("config/nmc_port")).toInt();
    emit SendChassisControl(NMCInitData);

    RobotChassis::ChassisControl NDC8InitData;
    NDC8InitData.ControlType_ = RobotChassis::ChassisControl::ControlType::NDC8_Init;
    NDC8InitData.strValue_ = configIni.value(QString("config/ndc8_ip")).toString().toStdString();
    NDC8InitData.iValue_ = configIni.value(QString("config/ndc8_port")).toInt();
    emit SendChassisControl(NDC8InitData);
}


// 点击向前按钮
void RobotChassisPage::on_forward_Button_clicked()
{
    RobotChassis::ChassisControl NMCInitData;
    NMCInitData.ControlType_ = RobotChassis::ChassisControl::ControlType::On_Forward;
    emit SendChassisControl(NMCInitData);
}

// 点击了向后的按钮
void RobotChassisPage::on_backwards_Button_clicked()
{
    RobotChassis::ChassisControl NMCInitData;
    NMCInitData.ControlType_ = RobotChassis::ChassisControl::ControlType::On_Backwards;
    emit SendChassisControl(NMCInitData);
}

// 点击了向左按钮
void RobotChassisPage::on_left_Button_clicked()
{
    RobotChassis::ChassisControl NMCInitData;
    NMCInitData.ControlType_ = RobotChassis::ChassisControl::ControlType::On_Left;
    emit SendChassisControl(NMCInitData);
}

// 点击了向右的按钮
void RobotChassisPage::on_right_Button_clicked()
{
    RobotChassis::ChassisControl NMCInitData;
    NMCInitData.ControlType_ = RobotChassis::ChassisControl::ControlType::On_Right;
    emit SendChassisControl(NMCInitData);
}

// 点击了停止按钮
void RobotChassisPage::on_stop_Button_clicked()
{
    RobotChassis::ChassisControl NMCInitData;
    NMCInitData.ControlType_ = RobotChassis::ChassisControl::ControlType::On_Stop;
    emit SendChassisControl(NMCInitData);
}

void RobotChassisPage::on_turnButton_clicked()
{
    RobotChassis::ChassisControl NMCInitData;
    NMCInitData.ControlType_ = RobotChassis::ChassisControl::ControlType::On_Turn;
    NMCInitData.dValue_ = (double)ui->angleEdit->text().toFloat();
    emit SendChassisControl(NMCInitData);
}

void RobotChassisPage::on_updata_lidarButton_clicked()
{
    RobotChassis::ChassisControl NMCInitData;
    NMCInitData.ControlType_ = RobotChassis::ChassisControl::ControlType::Get_Lidar;
    NMCInitData.dValue_ = (double)ui->angleEdit->text().toFloat();
    emit SendChassisControl(NMCInitData);
}

void RobotChassisPage::on_send_goal_Button_clicked()
{
    RobotChassis::ChassisControl NMCInitData;
    NMCInitData.ControlType_ = RobotChassis::ChassisControl::ControlType::Send_Goal;
    NMCInitData.iValue_ = (double)ui->goal_id_Edit->text().toInt();
    emit SendChassisControl(NMCInitData);
}
