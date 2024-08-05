#include "param_cfg_page.h"
#include "ui_param_cfg_page.h"
#include <QMouseEvent>

ParamCfgPage::ParamCfgPage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ParamCfgPage)
{
    ui->setupUi(this);
    ClickModelType_ = NotMode;
    ui->cmbPhotosPosture->addItem(u8"上");
    ui->cmbPhotosPosture->addItem(u8"下");
    ui->cmbPhotosPosture->setCurrentIndex(0);

    ui->lblRealTimeImage->setText("");
    QPalette palette;
    palette.setColor(QPalette::Window, QColor(200, 200, 200));
    ui->lblRealTimeImage->setAutoFillBackground(true);
    ui->lblRealTimeImage->setPalette(palette);
    ui->lblRealTimeImage->setMinimumSize(704, 396);
    ui->lblRealTimeImage->setMaximumSize(704, 396);

    LeftLimitImagePose_.SetZero();
    RightLimitImagePose_.SetZero();
}

ParamCfgPage::~ParamCfgPage()
{
    delete ui;
}

void ParamCfgPage::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        QPoint point = ui->lblRealTimeImage->mapFromGlobal(event->globalPos());
        if (ui->lblRealTimeImage->rect().contains(point))
        {
//            qDebug() << "图片像素点 ： " << qRound((1920.0/(double)ui->RealTimeImageLabel->width()) * point.x())
//                                        << qRound((1080.0/(double)ui->RealTimeImageLabel->height()) * point.y());
            qDebug() << "图片像素点 ： " << point.x() << point.y();
            if (ClickModelType_ == LeftLimitPoint)
            {
                ParameterConfigControl control_data;
//                ui->LeftLimitEdit->setText( QString::number(qRound((1920.0/(double)ui->RealTimeImageLabel->width()) * point.x())) );
                QuType_ = LeftLimitPoint;

                control_data.ControlType_ = ParameterConfigControl::ControlType::QuLimit;

                control_data.vecValue_.push_back(qRound((1920.0/(double)ui->lblRealTimeImage->width()) * point.x()));
                control_data.vecValue_.push_back(qRound((1080.0/(double)ui->lblRealTimeImage->height()) * point.y()));

                LeftLimitImagePose_.x_ = qRound((1920.0/(double)ui->lblRealTimeImage->width()) * point.x());
                LeftLimitImagePose_.y_ = qRound((1080.0/(double)ui->lblRealTimeImage->height()) * point.y());
                emit SetParameter(control_data);
            }
            if (ClickModelType_ == RightLimitPoint)
            {
                ParameterConfigControl control_data;
//                ui->RightLimitEdit->setText( QString::number(qRound((1920.0/(double)ui->RealTimeImageLabel->width()) * point.x())) );
                QuType_ = RightLimitPoint;

                control_data.ControlType_ = ParameterConfigControl::ControlType::QuLimit;

                control_data.vecValue_.push_back(qRound((1920.0/(double)ui->lblRealTimeImage->width()) * point.x()));
                control_data.vecValue_.push_back(qRound((1080.0/(double)ui->lblRealTimeImage->height()) * point.y()));

                RightLimitImagePose_.x_ = qRound((1920.0/(double)ui->lblRealTimeImage->width()) * point.x());
                RightLimitImagePose_.y_ = qRound((1080.0/(double)ui->lblRealTimeImage->height()) * point.y());
                emit SetParameter(control_data);
            }

            ClickModelType_ = NotMode;
        }
    }
    QWidget::mousePressEvent(event);
}

void ParamCfgPage::UpdateLiveStream(const cv::Mat& mat_data)
{
    if (mat_data.data == nullptr)
        {
            qDebug() << "mat_data.data == nullptr";
        }

        cv::Mat live_image;
        mat_data.copyTo(live_image);

    //    cv::putText(live_image, "TEST", cv::Point(80, 80), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 4, 8);

        cv::Point p1(Roi_.dRoi_x_, Roi_.dRoi_y_);
        cv::Point p2(Roi_.dRoi_x_ + Roi_.dRoi_w_, Roi_.dRoi_y_ + Roi_.dRoi_h_);
        cv::rectangle(live_image,p1,p2,cv::Scalar(255, 100, 100), 2);

        QImage neworiginalImage = QImage((const unsigned char*)(live_image.data), \
                                 live_image.cols, live_image.rows, \
                                 live_image.step, QImage::Format_RGB888);
        if (LeftLimitImagePose_.x_ != 0
            && LeftLimitImagePose_.y_ != 0)
        {
            cv::circle(live_image, cv::Point(LeftLimitImagePose_.x_, LeftLimitImagePose_.y_), 6, cv::Scalar(100, 100, 255),3);
        }

        if (RightLimitImagePose_.x_ != 0
            && RightLimitImagePose_.y_ != 0)
        {
            cv::circle(live_image, cv::Point(RightLimitImagePose_.x_, RightLimitImagePose_.y_), 6, cv::Scalar(100, 155, 200),3);
        }

        ui->lblRealTimeImage->setPixmap(QPixmap::fromImage(neworiginalImage.scaled(
                                           ui->lblRealTimeImage->width(), ui->lblRealTimeImage->height()
                                           , Qt::KeepAspectRatio
                                           , Qt::FastTransformation)));
}

void ParamCfgPage::Update(const RobotArm::RobotArmStateData &arm_data, const VisualIdentity::VisualIdentityStateData &visual_identity_data)
{
    if (CurrentTopRoi_ != visual_identity_data.TopROI_)
    {
        CurrentTopRoi_ = visual_identity_data.TopROI_;
    }

    if (CurrentBottomRoi_ != visual_identity_data.BottomROI_)
    {
        CurrentBottomRoi_ = visual_identity_data.BottomROI_;
    }

    if (dQuX_ != visual_identity_data.dQuX_
            || dQuY_ != visual_identity_data.dQuY_
            || dQuZ_ != visual_identity_data.dQuZ_)
    {
        dQuX_ = visual_identity_data.dQuX_;
        dQuY_ = visual_identity_data.dQuY_;
        dQuZ_ = visual_identity_data.dQuZ_;
        qDebug() << __FILE__ << "  " << __LINE__ << "  " << dQuX_ << " " << dQuY_ << " " << dQuZ_;

        Eigen::Matrix4d C_2_T3_Matrix;
        C_2_T3_Matrix  << 0.0233331,    -0.0870594, 0.99593,    3281.32 / 1000,
                          -0.0245225,   -0.995951,  -0.0864867, -604.529 / 1000,
                          0.999427,     -0.0224046, -0.0253735, 29.77 / 1000,
                          0,            0,          0,          1;
        Common::Pose pos(dQuX_, dQuY_, dQuZ_, 0.0, 0.0, 0.0);
        pos.TransformPose(C_2_T3_Matrix);
        std::vector<float> vecT3;
        vecT3.push_back(arm_data.fT3_x_ / 1000);
        vecT3.push_back(arm_data.fT3_y_ / 1000);
        vecT3.push_back(arm_data.fT3_z_ / 1000);
        vecT3.push_back(-arm_data.fT3_rx_);
        vecT3.push_back(arm_data.fT3_ry_);
        vecT3.push_back(arm_data.fT3_rz_);

        Eigen::Matrix4d T3_2_Base_Matrix = XYZRPYToMatrix4d(vecT3);
        pos.TransformPose(T3_2_Base_Matrix);
        pos.x_ *= 1000;
        pos.y_ *= 1000;
        pos.z_ *= 1000;
        qDebug() << "x:" << pos.x_ << "  y:" << pos.y_ << "  z:" << pos.z_;

        if (QuType_ == LeftLimitPoint)
        {
            ui->edtLeftLimit->setText(QString::number(pos.y_ - ((555/3) + 50) ));
        }
        else if (QuType_ == RightLimitPoint)
        {
            ui->edtRightLimit->setText(QString::number(pos.y_ + (555/3) + 50));
        }
    }

    if (arm_data.dLeftLimitPoint_ != dCurrentLeftLimitImagePose_)
    {
        dCurrentLeftLimitImagePose_ = arm_data.dLeftLimitPoint_;
        ui->edtCurrentLeftLimit->setText(QString::number(dCurrentLeftLimitImagePose_));
    }
    if (arm_data.dRightLimitPoint_ != dCurrentRightLimitImagePose_)
    {
        dCurrentRightLimitImagePose_ = arm_data.dRightLimitPoint_;
        ui->edtCurrentRightLimit->setText(QString::number(dCurrentRightLimitImagePose_));
    }

    ui->edtCurrentScoreValue->setText(QString::number(visual_identity_data.fScoreValue_));

    ui->edtCurrenLeftEdgeReturnAngle->setText(QString::number(arm_data.fLeftEdgeReturnAngle_));
    ui->edtCurrenLeftEdgeReturnDis->setText(QString::number(arm_data.fLeftEdgeReturnDis_));
    ui->edtCurrenLeftEdgeReturnXDis->setText(QString::number(arm_data.fLeftEdgeReturnXDis_));
    ui->edtCurrenRightEdgeReturnAngle->setText(QString::number(arm_data.fRightEdgeReturnAngle_));
    ui->edtCurrenRightEdgeReturnDis->setText(QString::number(arm_data.fRightEdgeReturnDis_));
    ui->edtCurrenRightEdgeReturnXDis->setText(QString::number(arm_data.fRightEdgeReturnXDis_));
}

// 获取左侧极限值的按钮
void ParamCfgPage::on_btnGetLeftLimitPoint_clicked()
{
    ClickModelType_ = LeftLimitPoint;
}

// 设置左侧极限值的按钮
void ParamCfgPage::on_btnSetLeftLimit_clicked()
{
    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::SetLeftLimit;
    control_data.vecValue_.push_back(ui->edtLeftLimit->text().toDouble());
    emit SetParameter(control_data);
}

// 获取右侧极限值的按钮
void ParamCfgPage::on_btnGetRightLimitPoint_clicked()
{
    ClickModelType_ = RightLimitPoint;
}

// 设置右侧极限值的按钮
void ParamCfgPage::on_btnSetRightLimit_clicked()
{
    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::SetRightLimit;
    control_data.vecValue_.push_back(ui->edtRightLimit->text().toDouble());
    emit SetParameter(control_data);
}

// 获取拍照点ROI的数据
void ParamCfgPage::on_btnGetPostureValue_clicked()
{
    if (ui->cmbPhotosPosture->currentIndex() == 0)
    {   // 上
        ui->edtROI_X->setText(QString::number(CurrentTopRoi_.dRoi_x_));
        ui->edtROI_Y->setText(QString::number(CurrentTopRoi_.dRoi_y_));
        ui->edtROI_Width->setText(QString::number(CurrentTopRoi_.dRoi_w_));
        ui->edtROI_Height->setText(QString::number(CurrentTopRoi_.dRoi_h_));
    }
    else if (ui->cmbPhotosPosture->currentIndex() == 1)
    {   // 下
        ui->edtROI_X->setText(QString::number(CurrentBottomRoi_.dRoi_x_));
        ui->edtROI_Y->setText(QString::number(CurrentBottomRoi_.dRoi_y_));
        ui->edtROI_Width->setText(QString::number(CurrentBottomRoi_.dRoi_w_));
        ui->edtROI_Height->setText(QString::number(CurrentBottomRoi_.dRoi_h_));
    }
    Roi_.dRoi_x_ = ui->edtROI_X->text().toDouble();
    Roi_.dRoi_y_ = ui->edtROI_Y->text().toDouble();
    Roi_.dRoi_w_ = ui->edtROI_Width->text().toDouble();
    Roi_.dRoi_h_ = ui->edtROI_Height->text().toDouble();
}

// 设置ROI的值
void ParamCfgPage::on_btnSetROI_clicked()
{
    Roi_.dRoi_x_ = ui->edtROI_X->text().toDouble();
    Roi_.dRoi_y_ = ui->edtROI_Y->text().toDouble();
    Roi_.dRoi_w_ = ui->edtROI_Width->text().toDouble();
    Roi_.dRoi_h_ = ui->edtROI_Height->text().toDouble();

    ParameterConfigControl control_data;
    if (ui->cmbPhotosPosture->currentIndex() == 0)
    {   // 上
        control_data.ControlType_ = ParameterConfigControl::ControlType::SetTopROI;
    }
    else if (ui->cmbPhotosPosture->currentIndex() == 1)
    {   // 下
        control_data.ControlType_ = ParameterConfigControl::ControlType::SetBottomROI;
    }
    control_data.Roi_.SetROI(ui->edtROI_X->text().toDouble()
                             , ui->edtROI_Y->text().toDouble()
                             , ui->edtROI_Width->text().toDouble()
                             , ui->edtROI_Height->text().toDouble());

    emit SetParameter(control_data);
}

// ROI 设置 移动到指定的拍照位
void ParamCfgPage::on_btnROIMoveArm_clicked()
{
    ParameterConfigControl control_data;
    if (ui->cmbPhotosPosture->currentIndex() == 0)
    {   // 上
        control_data.ControlType_ = ParameterConfigControl::ControlType::MoveArmToTop;
    }
    else if (ui->cmbPhotosPosture->currentIndex() == 1)
    {   // 下
        control_data.ControlType_ = ParameterConfigControl::ControlType::MoveArmToBottom;
    }
    emit SetParameter(control_data);
}

// 极限位置 移动机械臂到测试位
void ParamCfgPage::on_btnLinitMoveArm_clicked()
{
    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::MoveArmToLimitDetection;
    emit SetParameter(control_data);
}

void ParamCfgPage::on_btnOpenCamera_clicked()
{
    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::OpenCamera;
    emit SetParameter(control_data);
}

void ParamCfgPage::on_btnCloseCamera_clicked()
{
    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::CloseCamera;
    emit SetParameter(control_data);
}

void ParamCfgPage::on_btnSetScoreValue_clicked()
{
    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::SetScoreValue;
    control_data.fTempData_ = ui->edtSetScoreValue->text().toFloat();
    emit SetParameter(control_data);
}

// 贴边工艺参数  左侧-回归角度
void ParamCfgPage::on_btnSetLeftEdgeReturnAngle_clicked()
{
    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::LeftEdgeReturnAngle;
    control_data.fTempData_ = ui->edtSetLeftEdgeReturnAngle->text().toFloat();
    emit SetParameter(control_data);
}

// 贴边工艺参数  左侧-回归距离
void ParamCfgPage::on_btnSetLeftEdgeReturnDis_clicked()
{

    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::LeftEdgeReturnDis;
    control_data.fTempData_ = ui->edtSetLeftEdgeReturnDis->text().toFloat();
    emit SetParameter(control_data);
}

// 贴边工艺参数  左侧-X轴回归距离
void ParamCfgPage::on_btnSetLeftEdgeReturnXDis_clicked()
{

    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::LeftEdgeReturnXDis;
    control_data.fTempData_ = ui->edtSetLeftEdgeReturnXDis->text().toFloat();
    emit SetParameter(control_data);
}

// 贴边工艺参数  右侧-回归角度
void ParamCfgPage::on_btnSetRightEdgeReturnAngle_clicked()
{
    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::RightEdgeReturnAngle;
    control_data.fTempData_ = ui->edtSetRightEdgeReturnAngle->text().toFloat();
    emit SetParameter(control_data);
}

// 贴边工艺参数  右侧-回归距离
void ParamCfgPage::on_btnSetRightEdgeReturnDis_clicked()
{
    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::RightEdgeReturnDis;
    control_data.fTempData_ = ui->edtSetRightEdgeReturnDis->text().toFloat();
    emit SetParameter(control_data);
}

// 贴边工艺参数  右侧-X轴回归距离
void ParamCfgPage::on_btnSetRightEdgeReturnXDis_clicked()
{
    ParameterConfigControl control_data;
    control_data.ControlType_ = ParameterConfigControl::ControlType::RightEdgeReturnXDis;
    control_data.fTempData_ = ui->edtSetRightEdgeReturnXDis->text().toFloat();
    emit SetParameter(control_data);
}
