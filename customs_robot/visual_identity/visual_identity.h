#ifndef VISUALIDENTITY_H
#define VISUALIDENTITY_H
#include "work_base.h"
#include "common/common.h"
#include <vector>
#include <unordered_map>
#include <functional>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

namespace VisualIdentity{

enum VisualIdentityState
{
    Uninitialized = 0,      //  未初始化
    Normal = 1,             //  正常
    ERROR = 2               //  错误
};

class VisualIdentityStateData
{
public:
    VisualIdentityStateData()
    {
        VisualIdentityState_ = VisualIdentityState::Uninitialized;
        bArtificialData_ = true;
    }
    ~VisualIdentityStateData()
    {
    }

    VisualIdentityStateData& operator=(const VisualIdentityStateData& data)
    {
        this->vecBoxPose_ = data.vecBoxPose_;
        this->VisualIdentityState_ = data.VisualIdentityState_;
        this->bArtificialData_ = data.bArtificialData_;
        data.matOriginalImage_.copyTo(matOriginalImage_);
        data.matSegmentedImage_.copyTo(matSegmentedImage_);
        this->iwork_mode_ = data.iwork_mode_;
        TopROI_ = data.TopROI_;
        BottomROI_ = data.BottomROI_;
        dQuX_ = data.dQuX_;
        dQuY_ = data.dQuY_;
        dQuZ_ = data.dQuZ_;
        fScoreValue_ = data.fScoreValue_;
        return *this;
    }

public:
    VisualIdentityState VisualIdentityState_;   // 视觉识别类的状态
    std::vector<Common::Pose> vecBoxPose_;      // 视觉识别的结果
    std::vector<Common::Pose> vecUpLoadBoxPose_;// 视觉识别装货的位置
    bool bArtificialData_;
    int iwork_mode_;
    cv::Mat matOriginalImage_;
    cv::Mat matSegmentedImage_;
    double dValue_;
    Common::ROIRectangle TopROI_;
    Common::ROIRectangle BottomROI_;
    double dQuX_;
    double dQuY_;
    double dQuZ_;
    float fScoreValue_;
};


class VisualIdentityControl
{
public:
    enum ControlType
    {
        Init = 0,    // 用于初始化
        BoxDetection = 1,    // 界面触发单次箱体识别
        ConfirmVisualResults = 2,
        RetakeRecognition = 3
    };
    VisualIdentityControl()
    {
        iDataId_ = 0;
    }
    ~VisualIdentityControl(){}
public:
    ControlType ControlType_;
    std::vector<Common::Pose> vecBoxPoses_;
    int iDataId_;
    int imode_;
};


typedef std::function<void(const VisualIdentityStateData&)> STATE_CALLBACK;
typedef std::function<void(const cv::Mat&)> LIVE_CALLBACK;

class VisualIdentity : public CustomsRobot::WorkBase
{
public:
    VisualIdentity(STATE_CALLBACK call_back);
    ~VisualIdentity();

public:
    void SetROI(int artificial_region,const Common::ROIRectangle &roi_rectangle);
    void SetScoreValue(float score_value);
    void UV2RealPoint(int u, int v);
    // 开启人工模拟数据
    void EnabledArtificialData(bool data);
    // 激活视觉识别功能
    void CompteContainerOffset();
    void CompteBoxLoadPose();
    void ActivationIdentification();
    void ActivationIdentification(int identification_region);
    // 开始直播
    void StartLive(LIVE_CALLBACK live_callbask);
    // 停止直播
    void StopLive();
    void ChangeLiveType(const int& type);
    void ControlData(const VisualIdentityControl &data);
    // 重写任务线程
    void WorkFun();
private:
    void LiveThread();

private:
    VisualIdentityStateData VisualIdentityStateData_;   // 视觉识别模块状态
    STATE_CALLBACK pStateCallback_;                     // 状态回调函数
    bool bEnabledArtificialData_;                       // 是否开启人工模拟数据

public:
    bool bStartLive_;

private:    // 视频直播线程

    int iLiveType_;
    bool bIsStopLive_;                                  // 停止视频直播
    std::thread thLiveThread_;                          // 视频直播线程
    LIVE_CALLBACK pLiveCallback_;
    rs2::pipeline RsPipe_;
    std::unordered_map<int, std::vector<Common::Pose>> umapArtificialData_;   // 人工模拟数据 拍照姿势ID：模拟数据
    int iArtificialRegion_;
    int iwork_mode;

    int iQueryU_;
    int iQueryV_;
};

}

#endif // VISUALIDENTITY_H
