#ifndef ROBOT_CHASSIS_PAGE_H
#define ROBOT_CHASSIS_PAGE_H

#include <QWidget>
#include <QSettings>
#include "robot_chassis/robot_chassis.h"

namespace Ui {
class RobotChassisPage;
}

class RobotChassisPage : public QWidget
{
    Q_OBJECT

public:
    explicit RobotChassisPage(QWidget *parent = nullptr);
    ~RobotChassisPage();
     void Update(const RobotChassis::ChassisStateData &data);
     void LoadConfig(QSettings &configIni);

signals:
    void SendChassisControl(const RobotChassis::ChassisControl& control_data);

private slots:
    void on_forward_Button_clicked();

    void on_backwards_Button_clicked();

    void on_left_Button_clicked();

    void on_right_Button_clicked();

    void on_stop_Button_clicked();

    void on_turnButton_clicked();

    void on_updata_lidarButton_clicked();

    void on_send_goal_Button_clicked();

private:
    Ui::RobotChassisPage *ui;
};

#endif // ROBOT_CHASSIS_PAGE_H
