#ifndef ROBOT_ARM_PAGE_H
#define ROBOT_ARM_PAGE_H

#include <QButtonGroup>
#include <QWidget>
#include <QFileDialog>
#include <QSettings>
#include <QListWidgetItem>
#include <QCheckBox>
#include <unordered_map>
#include <QLabel>
#include "robot_arm/robot_arm.h"

namespace Ui {
class RobotArmPage;
}

class RobotArmPage : public QWidget
{
    Q_OBJECT
public:
    explicit RobotArmPage(QWidget *parent = nullptr);
    ~RobotArmPage();


    ZMC_HANDLE g_handle;
    QButtonGroup *pAxisGroup_;//axis group
    QButtonGroup *RunModeGroup;//run mode group

    bool m_out0 = false;
    bool m_out1 = false;
    bool m_out2 = false;
    bool m_out3 = false;
    bool m_outall = false;
    float fdpos13;
    int mode_14;
    int mode_12;
    int tubo_mode;
    int closenum;
    int m_nTimerId;
    uint32 IN9_status;
    uint32 IN10_status;
    uint32 IN11_status;
    int Robot_Status;          //正逆解状态
    int axis_real_list[6] = {0,1,2,3,4,5};
    int axis_rt_list[6] = {6,7,8,9,10,11};
public:
    void ip_scan();
    void up_State();
    void timerUpDate();
    void LoadConfig(QSettings &configIni);
    void Update(const RobotArm::RobotArmStateData &data);
signals:
    void SendRobotArmControl(const RobotArm::RobotArmControl& control_data);

private slots:
    void ClickedAxisGroup(int id);   // 轴单选框点击回调函数
//    void ClickedIoOutCheckBox(const QModelIndex &index);
    //-------todo--------

//    void timerEvent(QTimerEvent *event);

    void closeEvent(QCloseEvent *event);

    void on_pushButton_OpenEth_clicked();

    void on_pushButton_CloseEth_clicked();

    void on_btn_positive_clicked();

    void on_btn_inverse_clicked();

    void on_btn_real_move_clicked();

    void on_btn_Rt_clicked();

    void on_pushButton_stop_clicked();

    void on_btn_clear_clicked();

    void on_pushButton_RePos2_clicked();

    void on_btn_clear2_clicked();//单轴运动 编码器位置清零

//    void io_out0();

//    void io_out1();

//    void io_out2();

//    void io_out3();

//    void io_all_out();

    void on_tcp_tohome_clicked();

    void on_pushButton_RePos_clicked();

    void on_pushButton_Stop_clicked();

    void on_pushButton_Run_clicked();

    void on_stop_all_clicked();

    void on_tcp_Play_clicked();

    void tcp_single_run();

    void tcp_run();

    void base_run();

    void tubo_switch();

    void aixs12_run();

    void aixs14_run();

    void on_BasDown_clicked();

    void on_test_pushButton_clicked();

    void on_btn_CopyAxisReal_clicked();

    void on_btn_CopyAxisRt_clicked();

    void on_btn_Rapidstop_clicked();

    void on_tcp_push_clicked();

    void on_btn_getTcp_clicked();

    void on_btn_EnableUp_clicked();

    void on_btnStopRunConveyor_clicked();

    void on_btnStartRun_1_clicked();

    void on_btnStopRun_1_clicked();

    void on_btnStartRun_2_clicked();

    void on_btnStopRun_2_clicked();

    void on_btnStartRunConveyor_clicked();

    void on_btnEncircleMotion_clicked();

    void on_btnEncircleMotionStop_clicked();

private:
    Ui::RobotArmPage *ui;
    RobotArm::RobotArmControl ControlData_;
    std::unordered_map<int, QCheckBox*> umapIoOutCB_;
    std::unordered_map<int, QListWidgetItem*> umapIoInputItem_;
    std::unordered_map<int, QLabel*> umapAxisStatusText_;

    std::unordered_map<int, int> umapIoInput_;  //记录IO口的输入数值
    QList<QIcon> listStatusLampIcons_;
    RobotArm::RobotArmState RobotArmState_;
};


#endif // ROBOT_ARM_PAGE_H
