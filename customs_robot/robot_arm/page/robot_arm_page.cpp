#include "robot_arm_page.h"
#include "ui_robot_arm_page.h"
#include "robot_arm/robot_arm.h"
#include <QCoreApplication>
#include <QtCore>
#include <QMessageBox>
#include <QListWidgetItem>
#include <common/image_check_box.h>

RobotArmPage::RobotArmPage(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::RobotArmPage)
{
    ui->setupUi(this);
    g_handle = 0;
    //Axis group add and init set
    pAxisGroup_ = new QButtonGroup(this);
    pAxisGroup_->addButton(ui->radioButton_x,1);
    pAxisGroup_->addButton(ui->radioButton_y,2);
    pAxisGroup_->addButton(ui->radioButton_z,3);
    pAxisGroup_->addButton(ui->radioButton_r,4);
    pAxisGroup_->addButton(ui->radioButton_5,5);
    pAxisGroup_->addButton(ui->radioButton_6,14);
    pAxisGroup_->addButton(ui->radioButton_12,12);
    pAxisGroup_->addButton(ui->radioButton_13,13);
    pAxisGroup_->addButton(ui->radioButton_15,15);
//    connect(pAxisGroup_, SIGNAL(buttonClicked(int)), this, SLOT(ClickedAxisGroup(int)));
//    pAxisGroup_->buttonClicked(1);
    ui->radioButton_x->setChecked(true);//set init checked

    //-------todo-------

    closenum=1;
    ui->comboBox_IP->addItem("192.168.0.11");
    setWindowTitle("no line");
    m_nTimerId = -1;
    Robot_Status = 0; //正逆解状态

    ui->edit_State_L1->setText("0");
    ui->edit_State_L2->setText("0");
    ui->edit_State_L3->setText("0");
    ui->edit_State_L4->setText("0");
    ui->edit_State_L5->setText("0");
    ui->edit_State_V1->setText("0");
    ui->edit_State_V2->setText("0");
    ui->edit_State_V3->setText("0");
    ui->edit_State_V4->setText("0");
    ui->edit_State_V5->setText("0");
    ui->edit_State_V6->setText("0");
    ui->edit_Dis_L1->setText("0");
    ui->edit_Dis_L2->setText("0");
    ui->edit_Dis_L3->setText("0");
    ui->edit_Dis_L4->setText("0");
    ui->edit_Dis_L5->setText("0");
    ui->edit_Dis_L6->setText("0");
    ui->edit_Dis_V1->setText("0");
    ui->edit_Dis_V2->setText("0");
    ui->edit_Dis_V3->setText("0");
    ui->edit_Dis_V4->setText("0");
    ui->edit_Dis_V5->setText("0");
    ui->edit_Dis_V6->setText("0");
    ui->edit_mode->setText("请先选择正逆解模式");

    //Axis Paramter init
    ui->lineEdit_Units->setText("552730");
    ui->lineEdit_Lspeed->setText("0");
    ui->lineEdit_Speed->setText("50");
    ui->lineEdit_Accel->setText("1000");
    ui->lineEdit_Decel->setText("1000");
    ui->lineEdit_Sramp->setText("200");
    ui->lineEdit_inchdis->setText("10");
    //AxisStatue init
    ui->label_statue->setText("Idle : stop");
    ui->label_dpos->setText("Dpos : 0");
    ui->label_vspeed->setText("Vspeed : 0");

    listStatusLampIcons_ << QIcon("./config/icon/status_lamp/grey.png")
                         << QIcon("./config/icon/status_lamp/red.png")
                         << QIcon("./config/icon/status_lamp/green.png")
                         << QIcon("./config/icon/status_lamp/blue.png");
    ui->connect_label->setSizeIncrement(25,25);
    ui->connect_label->setPixmap(listStatusLampIcons_[0].pixmap(25,25));
    RobotArmState_ = RobotArm::RobotArmState::Uninitialized;

    //Run Mode group add and init set
    RunModeGroup = new QButtonGroup(this);
    RunModeGroup->addButton(ui->radioButton_continue,0);
    RunModeGroup->addButton(ui->radioButton_inch,1);
    ui->radioButton_continue->setChecked(true);//set init checked

    umapAxisStatusText_[0] = nullptr;
    umapAxisStatusText_[1] = nullptr;
    umapAxisStatusText_[2] = nullptr;
    umapAxisStatusText_[3] = nullptr;
    umapAxisStatusText_[4] = nullptr;
    umapAxisStatusText_[5] = nullptr;
    umapAxisStatusText_[6] = nullptr;
    umapAxisStatusText_[7] = nullptr;
    umapAxisStatusText_[8] = nullptr;
    umapAxisStatusText_[9] = nullptr;
    umapAxisStatusText_[10] = nullptr;
    umapAxisStatusText_[11] = nullptr;
    umapAxisStatusText_[12] = nullptr;
    umapAxisStatusText_[13] = nullptr;
   umapAxisStatusText_[14] = nullptr;
   umapAxisStatusText_[15] = nullptr;

    for (auto &itAxisStatus : umapAxisStatusText_)
    {
        QListWidgetItem *pListWAxisStatus = new QListWidgetItem();
        QString strStatus;
        strStatus = u8"轴" + QString::number(itAxisStatus.first);
        QLabel *qlabelAxisStatus = new QLabel();
        qlabelAxisStatus->setText(strStatus);
        ui->list_AxisStatus->addItem(pListWAxisStatus);
        ui->list_AxisStatus->setItemWidget(pListWAxisStatus , qlabelAxisStatus);
        itAxisStatus.second = qlabelAxisStatus;
    }

    // 暂时隐藏 按照时间控制
    ui->label_33->setVisible(false);
    ui->edtRunTime->setVisible(false);
    ui->checkRunConveyor_2->setVisible(false);
    ui->btnStartRun_2->setVisible(false);
    ui->btnStopRun_2->setVisible(false);
    ui->label_35->setVisible(false);
    ui->edtRemainingRunTime->setVisible(false);

//    ui->input0_label->setText("iput 8-0");
//    ui->input1_label->setText("iput 9-0");
//    ui->input2_label->setText("iput 10-0");
//    ui->input3_label->setText("iput 11-0");
//    connect(timer,SIGNAL(timeout()),this,SLOT(timerUpDate()));

    //输出口设置相应信号
//    connect(timer,SIGNAL(timeout()),this,SLOT(timerUpDate()));
}

RobotArmPage::~RobotArmPage()
{
    delete ui;
    if (pAxisGroup_)
    {
        delete pAxisGroup_;
    }
}


// 轴单选框点击回调函数
void RobotArmPage::ClickedAxisGroup(int id)
{
    RobotArm::RobotArmControl control_data;
    control_data.ControlType_ = RobotArm::RobotArmControl::ControlType::AxisGroupIndex;
    control_data.iAxisGroupIndex_ = id;
    emit SendRobotArmControl(control_data);
}


//void RobotArmPage::timerEvent(QTimerEvent *event)
//{

//    if(m_nTimerId == event->timerId())
//    {
//       // qDebug()<<"timer out!!!"<<m_nTimerId;
//        int idle;
//        float fdpos;
//        float fvspeed;
//        QString str_tmp;
//        ZAux_Direct_GetIfIdle(g_handle,pAxisGroup_->checkedId(),&idle);
//        str_tmp = QString("Idle : %1").arg(idle?"stop":"run");
//        ui->label_statue->setText(str_tmp);
//        //dpos
//        ZAux_Direct_GetDpos(g_handle,pAxisGroup_->checkedId(),&fdpos);
//        ZAux_Direct_GetDpos(g_handle,13,&fdpos13);
//        str_tmp = QString("Dpos : %1").arg(fdpos);
//        ui->label_dpos->setText(str_tmp);
//        //vspeed
//        ZAux_Direct_GetVpSpeed(g_handle,pAxisGroup_->checkedId(),&fvspeed);
//        str_tmp = QString("Vspeed : %1").arg(fvspeed);
//        ui->label_vspeed->setText(str_tmp);
//        up_State();


//        ZAux_Direct_GetDpos(g_handle,13,&fdpos13);
//        ZAux_Direct_GetIn(g_handle,9,&IN9_status);
//        ZAux_Direct_GetIn(g_handle,10,&IN10_status);
//        ZAux_Direct_GetIn(g_handle,11,&IN11_status);

//        tubo_switch();
//        aixs12_run();
//        aixs14_run();

//    }

//}
void RobotArmPage::closeEvent(QCloseEvent *event)
{
    (void)event;
    if(0 != g_handle)
    {
        ZAux_Close(g_handle);
        g_handle = NULL;
        setWindowTitle("no linK");
        killTimer(m_nTimerId);
        m_nTimerId=-1;
        closenum=0;
        qDebug()<<"close";
     }
    if(QMessageBox::No == QMessageBox::question(this,"提示","是否关闭"))
    {
        //event->ignore();
        closenum=0;
        return ;
    }

}


void RobotArmPage::ip_scan()
{

}

void RobotArmPage::aixs12_run()
{
    if(NULL == g_handle)
    {
        setWindowTitle("no link");
        return ;
    }

    if(mode_12 == 1) {
        if (fdpos13<15) {
            ZAux_Direct_SetSpeed(g_handle,12,450);
            ZAux_Direct_Single_Vmove(g_handle,12,1);
        }else {
            ZAux_Direct_SetSpeed(g_handle,12,150);
            ZAux_Direct_Single_Vmove(g_handle,12,1);
        }
    }
}

void RobotArmPage::aixs14_run() {
    if(NULL == g_handle)
    {
        setWindowTitle("no link");
        return ;
    }
    if(mode_14 == 1) {
        if (fdpos13<15) {
            ZAux_Direct_SetSpeed(g_handle,14,100);
            ZAux_Direct_Single_Vmove(g_handle,14,1);
        }else {
            ZAux_Direct_SetSpeed(g_handle,14,0);
            ZAux_Direct_Single_Vmove(g_handle,14,1);
        }
    }
}


void RobotArmPage::timerUpDate()//

{
    if(NULL == g_handle)
    {
        ZAux_Close(g_handle);
        return ;
    }
     else
     {
          uint32 iostatus[4] = {0};
          int i=0;
          for (i = 0;i< 4; i++)
           {
              ZAux_Direct_GetIn(g_handle,i+8,&iostatus[i]);
           }
      }
}

// 加载配置文件
void RobotArmPage::LoadConfig(QSettings &configIni)
{
    // 用于给机械臂业务模块初始化信息
    RobotArm::RobotArmControl init_data;
    init_data.ControlType_ = RobotArm::RobotArmControl::ControlType::InitIoOut;
    // 第一页 - 输出控制复选框 数据
    int io_out_num = configIni.value(QString("io_out/num"), 0).toUInt();
    for (int var = 0; var < io_out_num; ++var)
    {
        QString str_io_out(QString("io_out") + QString::number(var));
        int IoOutID = configIni.value(str_io_out + QString("/id")).toInt();
        QString IoOutName = configIni.value(str_io_out + QString("/name")).toString();

        init_data.umapIoData_[IoOutID] = IoOutName.toStdString(); // 将数据存进初始化的类中

        QListWidgetItem *pIoOutList = new QListWidgetItem();
        QCheckBox *pIoOutCB = new QCheckBox(IoOutName);
        umapIoOutCB_[IoOutID] = pIoOutCB;
        ui->IoOutList->addItem(pIoOutList);
        ui->IoOutList->setItemWidget(pIoOutList , pIoOutCB);
        connect(pIoOutCB, &QCheckBox::clicked,
                [this, IoOutID](bool state)
                {
                    if (IoOutID == 2000)
                    {
                        for (auto itIoOut : umapIoOutCB_)
                        {
                            if (itIoOut.first != IoOutID)
                            {
                                itIoOut.second->setChecked(state);
                            }
                        }
                    }
                    else
                    {
                        bool all_checked = true;
                        for (auto itIoOut : umapIoOutCB_)
                        {
                            if (itIoOut.first != 2000 && all_checked)
                            {
                                all_checked = itIoOut.second->isChecked() == all_checked ? true : false;
                            }
                        }
                        auto itIoOutAll = umapIoOutCB_.find(2000);
                        if (itIoOutAll != umapIoOutCB_.end())
                        {
                            itIoOutAll->second->setChecked(all_checked);
                        }
                    }
                    RobotArm::RobotArmControl control_data;
                    control_data.ControlType_ = RobotArm::RobotArmControl::ControlType::IoOut;
                    control_data.IoID_ = IoOutID;
                    control_data.IoState_ = state;
                    emit SendRobotArmControl(control_data);
                }
                );
    }
    emit SendRobotArmControl(init_data);    // 把初始化数据发给业务处理模块

    // 第一页 - 端口输入数据 数据
    int io_input_num = configIni.value(QString("io_input/num"), 0).toUInt();
    RobotArm::RobotArmControl init_input_data;
    init_input_data.ControlType_ = RobotArm::RobotArmControl::ControlType::InitIoInput;
    for (int var = 0; var < io_input_num; ++var)
    {
        QString str_io_input(QString("io_input") + QString::number(var));
        int IoInputId = configIni.value(str_io_input + QString("/id")).toInt();
        QString IoInputName = configIni.value(str_io_input + QString("/name")).toString();

        QListWidgetItem *pIoInputList = new QListWidgetItem(
                    configIni.value(str_io_input + QString("/name")).toString() + " 0");

        ui->IoInputList->addItem(pIoInputList);
        umapIoInputItem_[IoInputId] = pIoInputList;

        umapIoInput_[IoInputId] = 0;
        pIoInputList->setIcon(listStatusLampIcons_[1]);

        init_input_data.umapIoData_[IoInputId] = IoInputName.toStdString();
    }
    emit SendRobotArmControl(init_input_data);    // 把初始化数据发给业务处理模块
}

// 更新数据
void RobotArmPage::Update(const RobotArm::RobotArmStateData &data)
{
    QString strArmState;
    switch (data.RobotArmState_) {
        case RobotArm::RobotArmState::Normal :
        {
            strArmState = u8"通讯正常";
            break;
        }
        case RobotArm::RobotArmState::ArmIsError :
        {
            strArmState= u8"机械臂出错";
            break;
        }
        default:
        {
            strArmState = u8"未知";
            break;
        }
    }

    // 第一页
    for (auto itIoInput : data.umapIoInput_)
    {
//        umapIoInputItem_[itIoInput.first]->setIcon()

        if (umapIoInput_[itIoInput.first] != itIoInput.second)
        {
            umapIoInput_[itIoInput.first] = itIoInput.second;
            umapIoInputItem_[itIoInput.first]->setIcon(listStatusLampIcons_[itIoInput.second < 1 ? 1 : 2].pixmap(25,25));
        }
//        umapIoInputItem_[itIoInput.first]->setIcon(icons_[itIoInput.second < 1 ? 0 : 1]);
//        umapIoInputItem_[itIoInput.first]->setIcon(listStatusLampIcons_[2].pixmap(25,25));
        umapIoInputItem_[itIoInput.first]->setText(
                     QString(" IN %1 | %2").arg(itIoInput.first).arg(itIoInput.second));
    }

    // io状态
    for (auto itIoOut : data.umapIoOut_)
    {
        auto itIo = umapIoOutCB_.find(itIoOut.first);
        if (itIo != umapIoOutCB_.end())
        {
            itIo->second->setChecked(itIoOut.second);
        }
    }

    // 第一页
    ui->label_statue->setText(data.sAxisGroupIndexStatus_);
    ui->label_vspeed->setText(data.sAxisGroupIndexSpeed_);
    ui->label_dpos->setText(data.sAxisGroupIndexDpos_);

    if (data.RobotArmState_ == RobotArm::RobotArmState::Normal)
    {
        if (RobotArmState_ != data.RobotArmState_)
        {
            RobotArmState_ = data.RobotArmState_;
            ui->connect_label->setPixmap(listStatusLampIcons_[2].pixmap(25,25));
        }
    }
    else
    {
        if (RobotArmState_ != data.RobotArmState_)
        {
            RobotArmState_ = data.RobotArmState_;
            ui->connect_label->setPixmap(listStatusLampIcons_[1].pixmap(25,25));
        }
    }
    for (auto itAxisStatusString : data.umapAxisStatusString_)
    {
        if(umapAxisStatusText_[itAxisStatusString.first])
        {
            umapAxisStatusText_[itAxisStatusString.first]->setText(QString(itAxisStatusString.second.c_str()));
        }

    }
//    umapAxisStatusText_
    // 第二页 - 轴关节数据
    ui->edit_State_L1->setText(QString::number(data.fL_1_));
    ui->edit_State_L2->setText(QString::number(data.fL_2_));
    ui->edit_State_L3->setText(QString::number(data.fL_3_));
    ui->edit_State_L4->setText(QString::number(data.fL_4_));
    ui->edit_State_L5->setText(QString::number(data.fL_5_));
    ui->edit_State_L6->setText(QString::number(data.fL_6_));

    // 第二页 - TCP节数据
    ui->edit_State_V1->setText(QString::number(data.fTcp_x_));
    ui->edit_State_V2->setText(QString::number(data.fTcp_y_));
    ui->edit_State_V3->setText(QString::number(data.fTcp_z_));
    ui->edit_State_V4->setText(QString::number(data.fTcp_rx_));
    ui->edit_State_V5->setText(QString::number(data.fTcp_ry_));
    ui->edit_State_V6->setText(QString::number(data.fTcp_rz_));


    // 正逆解
    if (data.iRobotStatus_ == 1)
    {
        ui->edit_mode->setText(u8"正解模式");
    }
    else if (data.iRobotStatus_ == 2)
    {
        ui->edit_mode->setText(u8"逆解模式");
    }
    ui->LaserLongDist_label->setText(QString::number(data.dLaserLongDist_) + "mm");
    ui->LaserShortDist_label->setText(QString::number(data.dLaserShortDist_) + "mm");
    ui->LaserLeftDist_label->setText(QString::number(data.dLaserLeftDist_) + "mm");
    ui->LaserDownDist_label->setText(QString::number(data.dLaserDownDist_) + "mm");

    ui->edtRemainingRunCount->setText(QString::number(data.iTestModeRunCount_));

    ui->edtEncircleMotionRunCount->setText(QString::number(data.iTestModeEncircleMotionRunCount_));
//    data.umapIoInput_
//    ui->IoInputList

//    ui->input0_label->setText(data);

    //ui->ArmStateEdit->setText(strArmState);
}


void RobotArmPage::on_pushButton_OpenEth_clicked()//连接控制器
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::ConnectArm;
    emit SendRobotArmControl(data);
}

void RobotArmPage::up_State()
{
    //电机轴的状态
    float dis_real_list[6] = {0};
    for (int i = 0;i < 6;i++)
    {
        ZAux_Direct_GetDpos(g_handle,axis_real_list[i],&dis_real_list[i]);    //获取运动指令发送对位置，若想获取编码器的位置可以获取Mpos的值
    }
    ui->edit_State_L1->setText(QString("%1").arg(dis_real_list[0]));
    ui->edit_State_L2->setText(QString("%1").arg(dis_real_list[1]));
    ui->edit_State_L3->setText(QString("%1").arg(dis_real_list[2]));
    ui->edit_State_L4->setText(QString("%1").arg(dis_real_list[3]));
    ui->edit_State_L5->setText(QString("%1").arg(dis_real_list[4]));            //此处只显示了五个轴
    //虚拟轴对状态
    float dis_rt_list[6] = {0};
    for (int i = 0;i < 6;i++)
    {
        ZAux_Direct_GetDpos(g_handle,axis_rt_list[i],&dis_rt_list[i]);
    }
    ui->edit_State_V1->setText(QString("%1").arg(dis_rt_list[0]));
    ui->edit_State_V2->setText(QString("%1").arg(dis_rt_list[1]));
    ui->edit_State_V3->setText(QString("%1").arg(dis_rt_list[2]));
    ui->edit_State_V4->setText(QString("%1").arg(dis_rt_list[3]));
    ui->edit_State_V5->setText(QString("%1").arg(dis_rt_list[4]));            //此处只显示了五个轴
    ui->edit_State_V6->setText(QString("%1").arg(dis_rt_list[5]));
}

void RobotArmPage::on_pushButton_CloseEth_clicked()
{
    if(0 != g_handle)
    {
        ZAux_Close(g_handle);
        g_handle = NULL;
        ui->connect_label->setStyleSheet("border-image: url(:/png/discon.png)");
        setWindowTitle("no line");
        killTimer(m_nTimerId);
        m_nTimerId=-1;
     }
}

// 第一页 - 运动按钮
void RobotArmPage::on_pushButton_Run_clicked()
{
    RobotArm::RobotArmControl run_data;
    run_data.ControlType_ = RobotArm::RobotArmControl::ControlType::RunButton;
    run_data.iAxisGroupIndex_ = pAxisGroup_->checkedId();
    run_data.dUnits_ = ui->lineEdit_Units->text().toDouble();
    run_data.dSpeed_ = ui->lineEdit_Speed->text().toDouble();
    run_data.dAccel_ = ui->lineEdit_Accel->text().toDouble();
    run_data.dDecel_ = ui->lineEdit_Decel->text().toDouble();
    run_data.dSramp_ = ui->lineEdit_Sramp->text().toDouble();

    run_data.bMotorDirection_ = ui->checkBox_dir->checkState();

    run_data.iRunModeGroupIndex_ = RunModeGroup->checkedId();

    run_data.dMoveDistance_ = ui->lineEdit_inchdis->text().toDouble();
    emit SendRobotArmControl(run_data);
}

// 第一页 - 停止按钮
void RobotArmPage::on_pushButton_Stop_clicked()
{
    RobotArm::RobotArmControl stop_data;
    stop_data.ControlType_ = RobotArm::RobotArmControl::ControlType::StopButton;
    stop_data.iAxisGroupIndex_ = pAxisGroup_->checkedId();
    emit SendRobotArmControl(stop_data);
}

// 位置清零
void RobotArmPage::on_pushButton_RePos_clicked()
{
    RobotArm::RobotArmControl rest_data;
    rest_data.ControlType_ = RobotArm::RobotArmControl::ControlType::SingleAxisReset;
    rest_data.iAxisGroupIndex_ = pAxisGroup_->checkedId();
    emit SendRobotArmControl(rest_data);
}

void RobotArmPage::on_btn_positive_clicked()//正解模式
{
    RobotArm::RobotArmControl rt_data;
    rt_data.ControlType_ = RobotArm::RobotArmControl::ControlType::BtReal;
    emit SendRobotArmControl(rt_data);
}

void RobotArmPage::on_btn_inverse_clicked()//逆解模式
{
    RobotArm::RobotArmControl rt_data;
    rt_data.ControlType_ = RobotArm::RobotArmControl::ControlType::BtRt;
    emit SendRobotArmControl(rt_data);
}

void RobotArmPage::on_btn_real_move_clicked()//电机轴运动
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::RealRun;
    data.vecValue_.push_back(ui->edit_Dis_L1->text().toDouble());
    data.vecValue_.push_back(ui->edit_Dis_L2->text().toDouble());
    data.vecValue_.push_back(ui->edit_Dis_L3->text().toDouble());
    data.vecValue_.push_back(ui->edit_Dis_L4->text().toDouble());
    data.vecValue_.push_back(ui->edit_Dis_L5->text().toDouble());
    data.vecValue_.push_back(ui->edit_Dis_L6->text().toDouble());
    emit SendRobotArmControl(data);
//    if(g_handle == nullptr)
//    {
//        QMessageBox::warning(this,"警告","未链接到控制器");
//        return ;
//    }
//    if(Robot_Status == 2)
//    {
//        QMessageBox::warning(this,"警告","正处于逆解模式，不可操作电机轴");
//        return ;
//    }
//    if(Robot_Status == 0)
//    {
//        if(QMessageBox::No == QMessageBox::question(this,"警告","此时的运动并未建立正逆解，是否要运动"))
//        {
//            return ;//如果选择运动的话，此时只是单纯的轴控制
//        }
//    }
//    float dis[5]={0};
//    dis[0] = ui->edit_Dis_L1->text().toFloat();
//    dis[1] = ui->edit_Dis_L2->text().toFloat();
//    dis[2] = ui->edit_Dis_L3->text().toFloat();
//    dis[3] = ui->edit_Dis_L4->text().toFloat();
//    dis[4] = ui->edit_Dis_L5->text().toFloat();
//    ZAux_Direct_MoveAbs(g_handle,5,axis_real_list,dis);//从界面获取运动的目标位置，并以绝对位置对方式移动到目标位置，如果想用相对运动就使用Move
}

void RobotArmPage::on_btn_Rt_clicked()//虚拟轴运动
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::RtRun;
    data.vecValue_.push_back(ui->edit_Dis_V1->text().toDouble());
    data.vecValue_.push_back(ui->edit_Dis_V2->text().toDouble());
    data.vecValue_.push_back(ui->edit_Dis_V3->text().toDouble());
    data.vecValue_.push_back(ui->edit_Dis_V4->text().toDouble());
    data.vecValue_.push_back(ui->edit_Dis_V5->text().toDouble());
    data.vecValue_.push_back(ui->edit_Dis_V6->text().toDouble());
    emit SendRobotArmControl(data);
}

void RobotArmPage::on_pushButton_stop_clicked()
{
    if(g_handle == nullptr)
    {
        QMessageBox::warning(this,"警告","未链接到控制器");
        return ;
    }
    ZAux_Direct_Single_Cancel(g_handle,axis_real_list[0],2);
    ZAux_Direct_Single_Cancel(g_handle,axis_rt_list[0],2);
    ZAux_Direct_Singl_Cancel(g_handle,13,2);
}

void RobotArmPage::on_btn_clear_clicked() //位置清零
{
    if(g_handle == nullptr)
    {
        QMessageBox::warning(this,"警告","未链接到控制器");
        return ;
    }
    for (int i = 0;i < 5;i++)
    {
        ZAux_Direct_SetDpos(g_handle,axis_real_list[i],0);
        ZAux_Direct_SetDpos(g_handle,axis_rt_list[i],0);
    }
}

void RobotArmPage::on_pushButton_RePos2_clicked()//单轴运动 编码器位置清零
{
    RobotArm::RobotArmControl rest_coder_data;
    rest_coder_data.ControlType_ = RobotArm::RobotArmControl::ControlType::SingleAxisResetCoder;
    rest_coder_data.iAxisGroupIndex_ = pAxisGroup_->checkedId();
    emit SendRobotArmControl(rest_coder_data);
}

void RobotArmPage::on_btn_clear2_clicked()//多轴运动 编码器位置清零
{
    if(g_handle == nullptr)
    {
        QMessageBox::warning(this,"警告","未链接到控制器");
        return ;
    }
    for (int i = 0;i < 5;i++)
    {
        ZAux_Direct_SetMpos(g_handle,axis_real_list[i],0);
        ZAux_Direct_SetMpos(g_handle,axis_rt_list[i],0);
    }
}

void RobotArmPage::on_tcp_tohome_clicked()
{

    RobotArm::RobotArmControl rest_coder_data;
    rest_coder_data.ControlType_ = RobotArm::RobotArmControl::ControlType::TcpHoming;
    emit SendRobotArmControl(rest_coder_data);

//    if(NULL == g_handle)
//    {
//        setWindowTitle("no link");
//        return ;
//    }
//    int status = 0;
//    int m_datummode = 3;
//    ZAux_Direct_GetIfIdle(g_handle,13,&status);
//    if (status == 0) //已经在运动中
//        return;

//    ZAux_Direct_SetAtype(g_handle, 13, 65);
//    ZAux_Direct_SetUnits(g_handle, 13, 500000);
//    //设定脉冲模式及逻辑方向（脉冲+方向）
//    ZAux_Direct_SetInvertStep(g_handle, 13, 0);
//    ZAux_Direct_SetLspeed(g_handle, 13, 100);
//    ZAux_Direct_SetSpeed(g_handle, 13, 100);
//    ZAux_Direct_SetAccel(g_handle, 13, 1000);
//    ZAux_Direct_SetDecel(g_handle, 13, 1000);
//    ZAux_Direct_SetCreep(g_handle, 13, 100);
//    //设定对应轴的原点输入口信号
//    ZAux_Direct_SetDatumIn(g_handle, 13, 9);
//    ZAux_Direct_SetInvertIn(g_handle, 9, 0);	//ZMC系列认为OFF时碰到了原点信号（常闭）
//    ZAux_Direct_Singl_Datum(g_handle, 13, m_datummode );	//模式3回零
//    ZAux_Direct_SetDpos(g_handle,13,0);//设零点
//    ZAux_Direct_SetMpos(g_handle,13,0);
//    ZAux_Direct_SetRepDist(g_handle, 13, 500); //设置循环坐标为565
}


void RobotArmPage::on_stop_all_clicked()
{
    RobotArm::RobotArmControl rest_coder_data;
    rest_coder_data.ControlType_ = RobotArm::RobotArmControl::ControlType::StopAllAxis;
    emit SendRobotArmControl(rest_coder_data);

//    if(NULL == g_handle)
//    {
//        setWindowTitle("no link");
//        return ;
//    }
//    mode_14 = 0;
//    mode_12 = 0;
//    tubo_mode = 0;
//    for(int i = 0; i < 9; i++) {

//        ZAux_Direct_Singl_Cancel(g_handle, i+6, 2);
//    }

}

void RobotArmPage::tcp_single_run()
{
    if(0 == g_handle)
    {
        qDebug()<<"not link control!!";
        return;
    }

    int m_datummode=3;
    ZAux_Direct_SetAtype(g_handle,13,65);
    ZAux_Direct_SetUnits(g_handle,13,552730);

    ZAux_Direct_SetSpeed(g_handle,13,50);
    ZAux_Direct_SetAccel(g_handle,13,1000);
    ZAux_Direct_SetDecel(g_handle,13,1000);
    ZAux_Direct_SetCreep(g_handle, 13, 50);
    ZAux_Direct_SetMerge(g_handle,13,1);

    int axis_13=13;
    int GetValue13;
    int GetValue12;

    float dposlist1=37;
    float dposlist2=250;
    float dposlist3=250;
    float dposlist4=23;
    float dposlist15=-28;
    float dposlist16=-250;
    float dposlist17=-260;
    float dposlist18=-22;
    (void)dposlist18;



    ZAux_Direct_GetRemain_Buffer(g_handle,13,&GetValue13);
    ZAux_Direct_GetRemain_Buffer(g_handle,12,&GetValue12);
    if(GetValue13>0 && GetValue12>0) {
        ZAux_Direct_SetForceSpeed(g_handle,13,100);
        ZAux_Direct_SetEndMoveSpeed(g_handle,13,500);
        ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist1);//dposlist1=37;

        ZAux_Direct_SetForceSpeed(g_handle,13,500);
        ZAux_Direct_SetEndMoveSpeed(g_handle,13,500);
        ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist2);//dposlist2=250;

        ZAux_Direct_SetForceSpeed(g_handle,13,500);
        ZAux_Direct_SetEndMoveSpeed(g_handle,13,300);
        ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist3);//dposlist3=250;

        ZAux_Direct_SetForceSpeed(g_handle,13,30);
        ZAux_Direct_SetEndMoveSpeed(g_handle,13,10);
        ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist4);//dposlist4=23;

        ZAux_Direct_MoveDelay(g_handle,13,1000);

        //**********************************************

        ZAux_Direct_SetForceSpeed(g_handle,13,100);
        ZAux_Direct_SetEndMoveSpeed(g_handle,13,500);
        ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist15);//dposlist15=-28;


        ZAux_Direct_SetForceSpeed(g_handle,13,500);
        ZAux_Direct_SetEndMoveSpeed(g_handle,13,500);
        ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist16);//dposlist16=-250;


        ZAux_Direct_SetForceSpeed(g_handle,13,500);
        ZAux_Direct_SetEndMoveSpeed(g_handle,13,300);
        ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist17);//dposlist17=-260;


        ZAux_Direct_SetDatumIn(g_handle, 13, 9);
        ZAux_Direct_SetInvertIn(g_handle, 9, 0);	//ZMC系列认为OFF时碰到了原点信号（常闭）
        ZAux_Direct_Singl_Datum(g_handle, 13, m_datummode );	//模式3回零
        ZAux_Direct_SetDpos(g_handle,13,0);//设零点
        ZAux_Direct_SetMpos(g_handle,13,0);
        ZAux_Direct_SetRepDist(g_handle, 13, 500); //设置循环坐标为565
//        ZAux_Direct_SetForceSpeed(g_handle,13,100);
//        ZAux_Direct_SetEndMoveSpeed(g_handle,13,20);
//        ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist18);//dposlist18=-22;
        }
}

void RobotArmPage::tubo_switch()//电磁阀开关
{
    if(NULL == g_handle)
    {
        setWindowTitle("no link");
        return ;
    }
    if(tubo_mode == 1) {

        if (fdpos13<200) {
            ZAux_Direct_SetOp(g_handle,1,0);//关闭
            ZAux_Direct_SetOp(g_handle,0,0);
        }
        else if (fdpos13<500) {
            ZAux_Direct_SetOp(g_handle,1,1);//吹气
            ZAux_Direct_SetOp(g_handle,0,0);
        }else if (fdpos13<700) {
            ZAux_Direct_SetOp(g_handle,1,0);//吸气
            ZAux_Direct_SetOp(g_handle,0,1);
        }
    }

}

void RobotArmPage::on_tcp_Play_clicked()
{
    RobotArm::RobotArmControl rest_coder_data;
    rest_coder_data.ControlType_ = RobotArm::RobotArmControl::ControlType::TcpSingleRun;
    emit SendRobotArmControl(rest_coder_data);
//    tcp_single_run();
}

void RobotArmPage::tcp_run()//末端抓取
{
    if(0 == g_handle)
    {
        qDebug()<<"not link control!!";
        return;
    }

    int m_datummode=3;
    ZAux_Direct_SetAtype(g_handle,13,65);
    ZAux_Direct_SetAtype(g_handle,12,65);
    ZAux_Direct_SetUnits(g_handle,13,552730);
    ZAux_Direct_SetUnits(g_handle,12,552730);
    ZAux_Direct_SetSpeed(g_handle,13,50);
    ZAux_Direct_SetSpeed(g_handle,12,100);
    ZAux_Direct_SetAccel(g_handle,13,1000);
    ZAux_Direct_SetDecel(g_handle,13,1000);
    ZAux_Direct_SetCreep(g_handle, 13, 50);
    ZAux_Direct_SetAccel(g_handle,12,1000);
    ZAux_Direct_SetDecel(g_handle,12,1000);
    ZAux_Direct_SetMerge(g_handle,13,1);
    ZAux_Direct_SetMerge(g_handle,12,1);
    ZAux_Direct_SetDatumIn(g_handle, 13, 9);
    ZAux_Direct_SetInvertIn(g_handle, 9, 0);	//ZMC系列认为OFF时碰到了原点信号（常闭）
    ZAux_Direct_Singl_Datum(g_handle, 13, m_datummode );	//模式3回零
    ZAux_Direct_SetDpos(g_handle,13,0);//设零点
    ZAux_Direct_SetMpos(g_handle,13,0);
    ZAux_Direct_SetRepDist(g_handle, 13, 500); //设置循环坐标为565
    int axis_12=12;
    (void)axis_12;
    int axis_13=13;
    int GetValue13;
    int GetValue12;

    float dposlist1=37;
    float dposlist2=250;
    float dposlist3=250;

    float dposlist4=23;
    float dposlist15=-28;
    float dposlist16=-250;
    float dposlist17=-260;
    float dposlist18=-22;
    float dpos=30;
    (void)dpos;

    for (int i=0;i<9;i++) {
        ZAux_Direct_GetRemain_Buffer(g_handle,13,&GetValue13);
        ZAux_Direct_GetRemain_Buffer(g_handle,12,&GetValue12);
        if(GetValue13>0 && GetValue12>0) {
            ZAux_Direct_SetForceSpeed(g_handle,13,100);
            ZAux_Direct_SetEndMoveSpeed(g_handle,13,500);
            ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist1);//dposlist1=37;


            ZAux_Direct_SetForceSpeed(g_handle,13,500);
            ZAux_Direct_SetEndMoveSpeed(g_handle,13,500);
            ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist2);//dposlist2=250;


            ZAux_Direct_SetForceSpeed(g_handle,13,500);
            ZAux_Direct_SetEndMoveSpeed(g_handle,13,300);
            ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist3);//dposlist3=250;

            ZAux_Direct_SetForceSpeed(g_handle,13,30);
            ZAux_Direct_SetEndMoveSpeed(g_handle,13,10);
            ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist4);//dposlist4=23;

            ZAux_Direct_MoveDelay(g_handle,13,1000);

            //**********************************************

            ZAux_Direct_SetForceSpeed(g_handle,13,100);
            ZAux_Direct_SetEndMoveSpeed(g_handle,13,500);
            ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist15);//dposlist15=-28;


            ZAux_Direct_SetForceSpeed(g_handle,13,500);
            ZAux_Direct_SetEndMoveSpeed(g_handle,13,500);
            ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist16);//dposlist16=-250;


            ZAux_Direct_SetForceSpeed(g_handle,13,500);
            ZAux_Direct_SetEndMoveSpeed(g_handle,13,300);
            ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist17);//dposlist17=-260;

            ZAux_Direct_SetForceSpeed(g_handle,13,100);
            ZAux_Direct_SetEndMoveSpeed(g_handle,13,20);
            ZAux_Direct_MoveSp(g_handle,1,&axis_13,&dposlist18);//dposlist18=-22;


            ZAux_Direct_MoveDelay(g_handle,13,1000);

            }
        }
}

void RobotArmPage::base_run()
{
    if(g_handle == nullptr)
    {
        QMessageBox::warning(this,"警告","未链接到控制器");
        return ;
    }

    if(Robot_Status == 1)
    {
        QMessageBox::warning(this,"警告","正处于正解模式，不可操作虚拟轴");
        return ;
    }
    if(Robot_Status == 0)
    {
        if(QMessageBox::No == QMessageBox::question(this,"警告","此时的运动并未建立正逆解，是否要运动"))
        {
            return ;//如果选择运动的话，此时只是单纯的轴控制
        }
    }
    float array[10][6]={
                       {5440,350,1078,0,0,0},
                       {5440,-40,1078,0,0,0},
                       {5440,-430,1078,0,0,0},
                       {5440,-430,898,0,0,0},
                       {5440,-40,898,0,0,0},
                       {5440,350,898,0,0,0},
                       {5440,350,718,0,0,0},
                       {5440,-40,718,0,0,0},
                       {5440,-430,718,0,0,0},
                       {5417,0,1458,0,0,0},
                       };

    for (int i=0;i<10;i++) {
        float dis[6]={0};
        dis[0] = array[i][0];
        dis[1] = array[i][1];
        dis[2] = array[i][2];
        dis[3] = array[i][3];
        dis[4] = array[i][4];
        dis[5] = array[i][5];

        ZAux_Direct_MoveAbs(g_handle,6,axis_rt_list,dis);
        ZAux_Direct_MoveDelay(g_handle,6,4200);
    }
}

void RobotArmPage::on_BasDown_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::AxisEnableSignal;
    data.strData_ = "./config/enableInit.bas";
    emit SendRobotArmControl(data);
//    ZAux_BasDown(g_handle, filename, 0);

}

void RobotArmPage::on_test_pushButton_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::SaveTcpAndT3;
    emit SendRobotArmControl(data);
}

void RobotArmPage::on_btn_CopyAxisReal_clicked()
{
    ui->edit_Dis_L1->setText(ui->edit_State_L1->text());
    ui->edit_Dis_L2->setText(ui->edit_State_L2->text());
    ui->edit_Dis_L3->setText(ui->edit_State_L3->text());
    ui->edit_Dis_L4->setText(ui->edit_State_L4->text());
    ui->edit_Dis_L5->setText(ui->edit_State_L5->text());
    ui->edit_Dis_L6->setText(ui->edit_State_L6->text());
}

void RobotArmPage::on_btn_CopyAxisRt_clicked()
{
    ui->edit_Dis_V1->setText(ui->edit_State_V1->text());
    ui->edit_Dis_V2->setText(ui->edit_State_V2->text());
    ui->edit_Dis_V3->setText(ui->edit_State_V3->text());
    ui->edit_Dis_V4->setText(ui->edit_State_V4->text());
    ui->edit_Dis_V5->setText(ui->edit_State_V5->text());
    ui->edit_Dis_V6->setText(ui->edit_State_V6->text());
}


void RobotArmPage::on_btn_Rapidstop_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::Rapidstop;
    emit SendRobotArmControl(data);
}

void RobotArmPage::on_tcp_push_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::TcpPush;
    emit SendRobotArmControl(data);
}

void RobotArmPage::on_btn_getTcp_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::GetTcp;
    emit SendRobotArmControl(data);
}

void RobotArmPage::on_btn_EnableUp_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::AxisEnableSignal;
    data.strData_ = "./config/enableInit.bas";
    emit SendRobotArmControl(data);
}

// 大臂和背滚运动
void RobotArmPage::on_btnStopRunConveyor_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::TestMode_StopRunConveyor;
    emit SendRobotArmControl(data);
}

// _1 按照次数运行
void RobotArmPage::on_btnStartRun_1_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::TestMode_StartRunCount;
    data.vecValue_.push_back(ui->edtRunCount->text().toDouble());
    if (ui->checkRunConveyor_1->isChecked())
    {
        data.vecValue_.push_back(1.0);
    }
    else
    {
        data.vecValue_.push_back(-1.0);
    }
    emit SendRobotArmControl(data);
}

void RobotArmPage::on_btnStopRun_1_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::TestMode_StopRunCount;
    emit SendRobotArmControl(data);
}

// _2 按照时间运行
void RobotArmPage::on_btnStartRun_2_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::TestMode_StartRunTime;
    data.vecValue_.push_back(ui->edtRunTime->text().toDouble());
    if (ui->checkRunConveyor_2->isChecked())
    {
        data.vecValue_.push_back(1.0);
    }
    else
    {
        data.vecValue_.push_back(-1.0);
    }
    emit SendRobotArmControl(data);
}

void RobotArmPage::on_btnStopRun_2_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::TestMode_StopRunTime;
    emit SendRobotArmControl(data);
}

void RobotArmPage::on_btnStartRunConveyor_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::TestMode_StartRunConveyor;
    data.IoState_ = ui->radioUploadDirection->isChecked();
    emit SendRobotArmControl(data);
}

// 开始运行 环绕运动
void RobotArmPage::on_btnEncircleMotion_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::TestMode_StartEncircleMotion;
    emit SendRobotArmControl(data);
}

// 结束 环绕运动
void RobotArmPage::on_btnEncircleMotionStop_clicked()
{
    RobotArm::RobotArmControl data;
    data.ControlType_ = RobotArm::RobotArmControl::ControlType::TestMode_StopEncircleMotion;
    emit SendRobotArmControl(data);
}
