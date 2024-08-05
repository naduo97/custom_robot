#ifndef DIALOG_H
#define DIALOG_H

#include <unordered_map>
#include <QDialog>
#include <QSettings>
#include <QListWidget>
#include <QListWidgetItem>
#include <QStackedWidget>
#include "customs_robot.h"

#include "page/main_page.h"
#include "task_management/page/task_management_page.h"
#include "robot_chassis/page/robot_chassis_page.h"
#include "visual_identity/page/visual_identity_page.h"
#include "robot_arm/page/robot_arm_page.h"
#include "param_cfg_page.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Dialog; }
QT_END_NAMESPACE

class Dialog : public QDialog
{
    Q_OBJECT

public:
    Dialog(QWidget *parent = nullptr);
    ~Dialog();
public:
    bool ShowData(const CustomsRobot::RoboStateDate& state_data);
    void ErrorCallback(const std::string &error_data);
    void LiveStreamCallBack(const cv::Mat& mat_data);
    void RobotArmControlData(const RobotArm::RobotArmControl& control_data);
    void VisualIdentityControlData(const VisualIdentity::VisualIdentityControl& control_data);
    void ChassisControlData(const RobotChassis::ChassisControl& control_data);
protected:
//    virtual void resizeEvent(QResizeEvent *event) override;

private slots:
    void on_listWidget_itemClicked(QListWidgetItem *item);


private:
    void LoadININ();
    void Init();

private:
    bool bInit_;
    Ui::Dialog *ui;
//    QListWidget *pPageList_;
    CustomsRobot::Robot *pCustomsRobot_;
    QString strRobotParaFileName_;

private:
    std::unordered_map<std::string, int> umapPage_;
    MainPage *pMainPage_;
    TaskManagementPage *pTaskManagementPage_;
    RobotChassisPage *pRobotChassisPage_;
    RobotArmPage *pRobotArmPage_;
    VisualIdentityPage *pVisualIdentityPage_;
    ParamCfgPage *pParamCfgPage_;

private slots:
    void TaskControlData(const TaskManagement::TaskControl &data);
    void VisualIdentityEnabledArtificialData(bool data);
    void LiveStreamTypeChange(const int& type);
    void ParameterSetControlData(const ParameterConfigControl &data);

    void on_PauseOrResumeButton_clicked();
    void on_bt_Reset_clicked();
    void on_bt_Exit_clicked();
};
#endif // DIALOG_H
