#ifndef PARAM_CFG_PAGE_H
#define PARAM_CFG_PAGE_H

#include <QWidget>
#include "common/common.h"
#include "robot_arm/robot_arm.h"
#include "visual_identity/visual_identity.h"

class ParameterConfigControl
{
public:
    enum ControlType
    {
        SetTopROI = 0,
        SetBottomROI = 1,
        QuLimit = 2,
        SetLeftLimit = 3,
        SetRightLimit = 4,
        MoveArmToTop = 5,
        MoveArmToBottom = 6,
        MoveArmToLimitDetection = 7,
        OpenCamera = 8,
        CloseCamera = 9,
        SetScoreValue = 10,
        LeftEdgeReturnAngle = 11,
        LeftEdgeReturnDis = 12,
        LeftEdgeReturnXDis = 13,
        RightEdgeReturnAngle = 14,
        RightEdgeReturnDis = 15,
        RightEdgeReturnXDis = 16
    };
public:
    ParameterConfigControl(){}
    ~ParameterConfigControl(){}

public:
    ControlType ControlType_;
    std::vector<double> vecValue_;
    Common::ROIRectangle Roi_;
    float fTempData_;
};

enum ClickModelType
{
    NotMode = 0,
    LeftLimitPoint = 1,
    RightLimitPoint = 2
};

namespace Ui {
class ParamCfgPage;
}

class ParamCfgPage : public QWidget
{
    Q_OBJECT

public:
    explicit ParamCfgPage(QWidget *parent = nullptr);
    ~ParamCfgPage();

private:
    Ui::ParamCfgPage *ui;

signals:
    void SetParameter(const ParameterConfigControl &data);

public:
    void UpdateLiveStream(const cv::Mat& mat_data);
    void Update(const RobotArm::RobotArmStateData &arm_data, const VisualIdentity::VisualIdentityStateData &visual_identity_data);

    Eigen::Matrix4d XYZRPYToMatrix4d(std::vector<float> &xyzrpy)
    {
      Eigen::Matrix4d RT;
      double rx = xyzrpy[3]*M_PI/180;
      double ry = xyzrpy[4]*M_PI/180;
      double rz = xyzrpy[5]*M_PI/180;

      RT << cos(rz)*cos(ry), cos(rz)*sin(ry)*sin(rx)-sin(rz)*cos(rx), cos(rz)*sin(ry)*cos(rx)+sin(rz)*sin(rx), xyzrpy[0],
            sin(rz)*cos(ry), sin(rz)*sin(ry)*sin(rx)+cos(rz)*cos(rx), sin(rz)*sin(ry)*cos(rx)-cos(rz)*sin(rx), xyzrpy[1],
                   -sin(ry),                         cos(ry)*sin(rx),                         cos(ry)*cos(rx), xyzrpy[2],
                          0,                                       0,                                       0,         1;
      return RT;
    }

protected:
    virtual void mousePressEvent(QMouseEvent *event) override;

private slots:
    void on_btnGetLeftLimitPoint_clicked();     // 获取左侧极限值的按钮
    void on_btnSetLeftLimit_clicked();          // 设置左侧极限值的按钮
    void on_btnGetRightLimitPoint_clicked();    // 获取右侧极限值的按钮
    void on_btnSetRightLimit_clicked();         // 设置右侧极限值的按钮
    void on_btnGetPostureValue_clicked();       // 获取拍照点ROI的数据
    void on_btnSetROI_clicked();                // 设置ROI的值

    void on_btnROIMoveArm_clicked();

    void on_btnLinitMoveArm_clicked();

    void on_btnOpenCamera_clicked();

    void on_btnCloseCamera_clicked();

    void on_btnSetScoreValue_clicked();

    void on_btnSetLeftEdgeReturnAngle_clicked();

    void on_btnSetLeftEdgeReturnDis_clicked();

    void on_btnSetLeftEdgeReturnXDis_clicked();

    void on_btnSetRightEdgeReturnAngle_clicked();

    void on_btnSetRightEdgeReturnDis_clicked();

    void on_btnSetRightEdgeReturnXDis_clicked();

private:
    Common::ROIRectangle CurrentTopRoi_;
    Common::ROIRectangle CurrentBottomRoi_;
    Common::ROIRectangle Roi_;

    ClickModelType ClickModelType_;

    ClickModelType QuType_;

    double dQuX_;
    double dQuY_;
    double dQuZ_;

    Common::Pose LeftLimitImagePose_;
    Common::Pose RightLimitImagePose_;

    double dCurrentLeftLimitImagePose_;
    double dCurrentRightLimitImagePose_;
};

#endif // PARAM_CFG_PAGE_H
