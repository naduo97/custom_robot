#include "visual_identity_page.h"
#include "ui_visual_identity_page.h"

VisualIdentityPage::VisualIdentityPage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::VisualIdentityPage)
{
    ui->setupUi(this);
    // 图片显示
    ui->originalImage->clear();
    QPalette palette;
    palette.setColor(QPalette::Window, QColor(200, 200, 200));
    ui->originalImage->setAutoFillBackground(true);
    ui->originalImage->setPalette(palette);
    ui->originalImage->setMinimumSize(336, 189);

    ui->segmentedImage->clear();
    ui->segmentedImage->setAutoFillBackground(true);
    ui->segmentedImage->setPalette(palette);
    ui->segmentedImage->setMinimumSize(336, 189);

    // 单选框
    ui->artificialData->setChecked(true);

    // 结果列表
    ui->resultTable->setColumnCount(7);
    ui->resultTable->setRowCount(6);
    ui->resultTable->setFocusPolicy(Qt::NoFocus);
    ui->resultTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->resultTable->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->resultTable->resizeRowsToContents();
    ui->resultTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->resultTable->verticalHeader()->setHidden(true);
    QStringList headerText;
    headerText<< u8"箱体ID" << "x" << "y" << "z" << "pitch" << "roll" << "yaw";
    ui->resultTable->setHorizontalHeaderLabels(headerText);

    ui->comboBox_DetectionType->addItem("1");
    ui->comboBox_DetectionType->addItem("2");
}

VisualIdentityPage::~VisualIdentityPage()
{
    delete ui;
}

// 加载配置
void VisualIdentityPage::LoadConfig(QSettings &configIni)
{
    (void)configIni;
    int box_num = configIni.value(QString("box/num"), 0).toUInt();
    int box_type_id = configIni.value(QString("box/type_id"), 0).toUInt();
    int mode_id = configIni.value(QString("box/mode"), 0).toUInt();//mode=1为卸货，mode＝2为装货
    VisualIdentity::VisualIdentityControl InitData;
    InitData.ControlType_ = VisualIdentity::VisualIdentityControl::Init;
    InitData.iDataId_ = box_type_id;
    InitData.imode_ = mode_id;
    for (int var = 0; var < box_num; ++var)
    {
        InitData.vecBoxPoses_.push_back(Common::Pose(
            configIni.value(QString("box") + QString::number(var) + QString("/x"), 0).toDouble()
          , configIni.value(QString("box") + QString::number(var) + QString("/y"), 0).toDouble()
          , configIni.value(QString("box") + QString::number(var) + QString("/z"), 0).toDouble()
          , configIni.value(QString("box") + QString::number(var) + QString("/pitch"), 0).toDouble()
          , configIni.value(QString("box") + QString::number(var) + QString("/roll"), 0).toDouble()
          , configIni.value(QString("box") + QString::number(var) + QString("/yaw"), 0).toDouble()));
    }
    emit SendVisualIdentityControl(InitData);
}

// 更新数据
void VisualIdentityPage::Update(const VisualIdentity::VisualIdentityStateData &data)
{
    if (ui->artificialData->isChecked() != data.bArtificialData_)
    {
        ui->artificialData->setChecked(data.bArtificialData_);
    }
    int tempID = 1;
    ui->resultTable->clearContents();
    ui->resultTable->setRowCount(data.vecBoxPose_.size());
    for (auto itPose : data.vecBoxPose_)
    {
        ui->resultTable->setItem(tempID-1, 0, new QTableWidgetItem(std::to_string(tempID).c_str()));
        ui->resultTable->setItem(tempID-1, 1, new QTableWidgetItem(std::to_string(itPose.x_).c_str()));
        ui->resultTable->setItem(tempID-1, 2, new QTableWidgetItem(std::to_string(itPose.y_).c_str()));
        ui->resultTable->setItem(tempID-1, 3, new QTableWidgetItem(std::to_string(itPose.z_).c_str()));
        ui->resultTable->setItem(tempID-1, 4, new QTableWidgetItem(std::to_string(itPose.pitch_).c_str()));
        ui->resultTable->setItem(tempID-1, 5, new QTableWidgetItem(std::to_string(itPose.roll_).c_str()));
        ui->resultTable->setItem(tempID-1, 6, new QTableWidgetItem(std::to_string(itPose.yaw_).c_str()));
        tempID++;
    }
    if (data.vecBoxPose_.size() == 0 && ui->resultTable->rowCount() > 1)
    {
        ui->resultTable->clearContents();
    }

    if (data.vecBoxPose_.size() != 0)
    {
        QImage neworiginalImage = QImage((const unsigned char*)(data.matOriginalImage_.data), \
                                 data.matOriginalImage_.cols, data.matOriginalImage_.rows, \
                                 data.matOriginalImage_.step, QImage::Format_RGB888);
        ui->originalImage->setPixmap(QPixmap::fromImage(\
            neworiginalImage.scaled(ui->originalImage->width(), ui->originalImage->height(), \
            Qt::KeepAspectRatio, Qt::FastTransformation)));

        QImage newSegmentedImage = QImage((const unsigned char*)(data.matSegmentedImage_.data), \
                                  data.matSegmentedImage_.cols, data.matSegmentedImage_.rows, \
                                  data.matSegmentedImage_.step, QImage::Format_RGB888);
        ui->segmentedImage->setPixmap(QPixmap::fromImage( \
             newSegmentedImage.scaled(ui->segmentedImage->width(), ui->segmentedImage->height(), \
             Qt::KeepAspectRatio, Qt::FastTransformation)));
    }
    else
    {
        ui->originalImage->clear();
        ui->segmentedImage->clear();
    }

}

void VisualIdentityPage::on_artificialData_clicked()
{
    emit ModifyingArtificialData(ui->artificialData->isChecked());
}

void VisualIdentityPage::on_btn_BoxDetection_clicked()
{
    VisualIdentity::VisualIdentityControl InitData;
    InitData.ControlType_ = VisualIdentity::VisualIdentityControl::BoxDetection;
//    ui->taskComboBox->currentText().toUtf8().toStdString()
    InitData.iDataId_ = std::stoi(ui->comboBox_DetectionType->currentText().toUtf8().toStdString().c_str());
    emit SendVisualIdentityControl(InitData);
}

// 确定识别的结果
void VisualIdentityPage::on_btnConfirmVisualResults_clicked()
{
    VisualIdentity::VisualIdentityControl ControlData;
    ControlData.ControlType_ = VisualIdentity::VisualIdentityControl::ConfirmVisualResults;
    emit SendVisualIdentityControl(ControlData);
}

// 重拍照片并重新识别
void VisualIdentityPage::on_btnRetakeRecognition_clicked()
{
    VisualIdentity::VisualIdentityControl ControlData;
    ControlData.ControlType_ = VisualIdentity::VisualIdentityControl::RetakeRecognition;
    emit SendVisualIdentityControl(ControlData);
}
