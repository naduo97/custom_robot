#include "task_management_page.h"
#include "ui_task_management_page.h"
#include <QDebug>

TaskManagementPage::TaskManagementPage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TaskManagementPage)
{
    ui->setupUi(this);
    //    ui->tableWidget->setColumnCount(4);
    ui->taskTableWidget->setColumnCount(10);
    ui->taskTableWidget->setRowCount(1);
    ui->taskTableWidget->setFocusPolicy(Qt::NoFocus);
    ui->taskTableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    //    ui->taskTableWidget->resizeColumnsToContents();
    ui->taskTableWidget->resizeRowsToContents();
    ui->taskTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->taskTableWidget->verticalHeader()->setHidden(true);
    QStringList headerText;
    headerText<< u8"名称" << u8"车厢ID" << u8"工作类型" << "x" << "y" << "z" << "qx" << "qy" << "qz" << "qw";
                  ui->taskTableWidget->setHorizontalHeaderLabels(headerText);

    ui->taskTableWidget->setItem(0, 0, umapListData_[0] = new QTableWidgetItem(""));
    ui->taskTableWidget->setItem(0, 1, umapListData_[1] = new QTableWidgetItem(QString::number(0)));
    ui->taskTableWidget->setItem(0, 2, umapListData_[2] = new QTableWidgetItem(QString::number(0)));
    ui->taskTableWidget->setItem(0, 3, umapListData_[3] = new QTableWidgetItem(QString::number(0)));
    ui->taskTableWidget->setItem(0, 4, umapListData_[4] = new QTableWidgetItem(QString::number(0)));
    ui->taskTableWidget->setItem(0, 5, umapListData_[5] = new QTableWidgetItem(QString::number(0)));
    ui->taskTableWidget->setItem(0, 6, umapListData_[6] = new QTableWidgetItem(QString::number(0)));
    ui->taskTableWidget->setItem(0, 7, umapListData_[7] = new QTableWidgetItem(QString::number(0)));
    ui->taskTableWidget->setItem(0, 8, umapListData_[8] = new QTableWidgetItem(QString::number(0)));
    ui->taskTableWidget->setItem(0, 9, umapListData_[9] = new QTableWidgetItem(QString::number(0)));

}

TaskManagementPage::~TaskManagementPage()
{
    ui->taskTableWidget->clearContents();
    delete ui;
}

void TaskManagementPage::LoadConfig(QSettings &configIni)
{
    int task_num = configIni.value(QString("tasks/num"), 0).toUInt();
    for (int var = 1; var <= task_num; ++var)
    {
        QString str_task(QString("task") + QString::number(var));
        TaskManagement::TaskData temp_task_data;
        temp_task_data.strText_ = configIni.value(str_task + QString("/text") , "").toString().toUtf8().toStdString();
        temp_task_data.iCarriageId_ = configIni.value(str_task + QString("/carriage_id") , 0).toInt();
        temp_task_data.iWorkType_ = configIni.value(str_task + QString("/work_type") , 0).toInt();
        temp_task_data.Pose_.x_ = configIni.value(str_task + QString("/x") , 0.0).toDouble();
        temp_task_data.Pose_.y_ = configIni.value(str_task + QString("/y") , 0.0).toDouble();
        temp_task_data.Pose_.x_ = configIni.value(str_task + QString("/z") , 0.0).toDouble();
        temp_task_data.Pose_.qx_ = configIni.value(str_task + QString("/qx") , 0.0).toDouble();
        temp_task_data.Pose_.qy_ = configIni.value(str_task + QString("/qy") , 0.0).toDouble();
        temp_task_data.Pose_.qz_ = configIni.value(str_task + QString("/qz") , 0.0).toDouble();
        temp_task_data.Pose_.qw_ = configIni.value(str_task + QString("/qw") , 0.0).toDouble();
        umapTaskData_[temp_task_data.strText_] = temp_task_data;

        ui->taskComboBox->addItem(temp_task_data.strText_.c_str());
    }
}

void TaskManagementPage::Update(const TaskManagement::TaskStateData &data)
{
    umapListData_[0]->setText(data.CurrentTask_.strText_.c_str());
    umapListData_[1]->setText(QString::number(data.CurrentTask_.iCarriageId_));
    umapListData_[2]->setText(QString::number(data.CurrentTask_.iWorkType_));
    umapListData_[3]->setText(QString::number(data.CurrentTask_.Pose_.x_));
    umapListData_[4]->setText(QString::number(data.CurrentTask_.Pose_.y_));
    umapListData_[5]->setText(QString::number(data.CurrentTask_.Pose_.z_));
    umapListData_[6]->setText(QString::number(data.CurrentTask_.Pose_.qx_));
    umapListData_[7]->setText(QString::number(data.CurrentTask_.Pose_.qy_));
    umapListData_[8]->setText(QString::number(data.CurrentTask_.Pose_.qz_));
    umapListData_[9]->setText(QString::number(data.CurrentTask_.Pose_.qw_));

    if (data.TaskState_ == 1)
    {
        ui->taskState->setText("正常");
    }
    else
    {
        ui->taskState->setText("错误");
    }
}


void TaskManagementPage::Init(QSettings &config)
{
    (void)config;
}

void TaskManagementPage::on_taskPushButton_clicked()
{
    TaskManagement::TaskControl ControlData;
    ControlData.ControlType_ = TaskManagement::TaskControl::ControlType::PublishTask;
    ControlData.TaskData_ = umapTaskData_[ui->taskComboBox->currentText().toUtf8().toStdString()];
//    std::string str = ui->taskComboBox->currentText().toUtf8().toStdString();

    emit SendTaskData(ControlData);
}


void TaskManagementPage::on_complete_mountingButton_clicked()
{
    TaskManagement::TaskControl ControlData;
    ControlData.ControlType_ = TaskManagement::TaskControl::CompleteMounting;

    emit SendTaskData(ControlData);
}

void TaskManagementPage::on_complete_unmountingButton_clicked()
{
    TaskManagement::TaskControl ControlData;
    ControlData.ControlType_ = TaskManagement::TaskControl::CompleteUnMounting;

    emit SendTaskData(ControlData);
}

// 完成装货按钮
void TaskManagementPage::on_complete_loadingButton_clicked()
{
    TaskManagement::TaskControl ControlData;
    ControlData.ControlType_ = TaskManagement::TaskControl::CompleteLoading;
    emit SendTaskData(ControlData);
}
