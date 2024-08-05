#include "visual_identity.h"
#include <QDebug>
#include <QImage>
#include <QMessageBox>

#ifdef CODE_VERSION_VISUAL
#include "objDetection.h"
#endif

namespace VisualIdentity{
VisualIdentity::VisualIdentity(STATE_CALLBACK call_back)
    :CustomsRobot::WorkBase(CustomsRobot::WhileType::SEMAPHORE)
{
    iArtificialRegion_ = 0;
    iLiveType_ = 0;
    bIsStopLive_ = false;
    bEnabledArtificialData_ = false;
    pStateCallback_ = call_back;
    pLiveCallback_ = nullptr;

    VisualIdentityStateData_.TopROI_.SetROI(350, 560, 1340,425);// 500, 300, 1060, 660);
    VisualIdentityStateData_.BottomROI_.SetROI(370, 347, 1270, 410);// (230, 260, 1520, 560);
    VisualIdentityStateData_.fScoreValue_ = 0.83;

    #ifdef CODE_VERSION_MAIN
        bStartLive_ = false;
    #elif CODE_VERSION_VISUAL
        bStartLive_ = true;
    #endif

    // 初始化 摄像头直播
    rs2::context ctx;
    auto devs = ctx.query_devices();                  ///获取设备列表
    if (devs.size() == 0)
    {
        VisualIdentityStateData_.VisualIdentityState_ = VisualIdentityState::ERROR;
        pStateCallback_(VisualIdentityStateData_);
        if (bStartLive_)    //开启直播
        {
            exit(1);
        }
    }
    else
    {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);
        RsPipe_.start(cfg);
    }

    WorkBase::ExecutionThread();
    VisualIdentityStateData_.VisualIdentityState_ = VisualIdentityState::Normal;
    VisualIdentityStateData_.bArtificialData_ = bEnabledArtificialData_;
    pStateCallback_(VisualIdentityStateData_);
}

VisualIdentity::~VisualIdentity()
{
    WorkBase::Stop();
    pStateCallback_ = nullptr;
    pLiveCallback_ = nullptr;
    StopLive();
    if (thLiveThread_.joinable())
    {
        thLiveThread_.join();
    }
    if (bStartLive_)
    {
        RsPipe_.stop();
    }
}

void VisualIdentity::WorkFun()
{
    qDebug() << "VisualIdentity::WorkFun" << "->" << iwork_mode;
    // 调用视觉库
    // 清楚内存中的数据
    VisualIdentityStateData_.vecBoxPose_.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));   // 2s

    if (iwork_mode == 1)    // 卸货
    {
        // 填充人工的模拟数据
        if (bEnabledArtificialData_)
        {
            auto itArtificialData = umapArtificialData_.find(iArtificialRegion_);
            if (itArtificialData != umapArtificialData_.end())
            {
                for (auto &itPoseData : itArtificialData->second)
                {
                    VisualIdentityStateData_.vecBoxPose_.push_back(itPoseData);
                }
            }
        }

        qDebug() << "VisualIdentity::WorkFun1111111111";
        try {
            rs2::frameset frames = RsPipe_.wait_for_frames();
    //        rs2::depth_frame depth_frame = frames.get_depth_frame();
            rs2::video_frame color_frame = frames.get_color_frame();

            rs2::align align_to_color(RS2_STREAM_COLOR);
            auto aligned_frames = align_to_color.process(frames);
            auto depth_frame = aligned_frames.get_depth_frame();

            int cw = color_frame.as<rs2::video_frame>().get_width();
            int ch = color_frame.as<rs2::video_frame>().get_height();

            VisualIdentityStateData_.matOriginalImage_ = cv::Mat(cv::Size(cw,ch), CV_8UC3);
            memcpy(VisualIdentityStateData_.matOriginalImage_.data, (void*)color_frame.get_data(), cw * ch * 3);
            cv::cvtColor(VisualIdentityStateData_.matOriginalImage_, VisualIdentityStateData_.matOriginalImage_, cv::COLOR_BGR2RGB);


            cv::Mat temp_depth(depth_frame.get_height(), depth_frame.get_width(), CV_16UC1, (void*)depth_frame.get_data());

            // 视觉检测
            double fx = 1349.66, fy = 1349.17, cx = 982.689, cy = 545.792;
            Eigen::MatrixXd K(3,3);
            Eigen::VectorXd distCoeffs(5);
            K << fx,0,cx,0,fy,cy,0,0,1;
            distCoeffs << 0,0,0,0,0;
            cv::Mat temp_color = cv::Mat(cv::Size(cw,ch), CV_8UC3);
            memcpy(temp_color.data, (void*)color_frame.get_data(), cw * ch * 3);
            cv::cvtColor(temp_color, temp_color, cv::COLOR_BGR2RGB);
            cv::cvtColor(temp_color, temp_color, cv::COLOR_RGB2BGR);
    //        cv::cvtColor(temp_color, temp_color, cv::COLOR_BGR2RGB);
            cv::imwrite("in_color_image.png",temp_color);
            cv::imwrite("in_depth_image.png",temp_depth);

    //        cv::Mat temp_depth = cv::Mat(cv::Size(cw,ch), CV_8UC3);
    //        memcpy(temp_depth.data, (void*)color_map.colorize(depth_frame).get_data(), dw * dh * 3);

            qDebug() << "VisualIdentity::WorkFun333333333333";
            float roi_x = 0;
            float roi_y = 0;
            float roi_w = 0;
            float roi_h = 0;

            if (iArtificialRegion_ == 1)
            {
                roi_x = VisualIdentityStateData_.TopROI_.dRoi_x_;
                roi_y = VisualIdentityStateData_.TopROI_.dRoi_y_;
                roi_w = VisualIdentityStateData_.TopROI_.dRoi_w_;
                roi_h = VisualIdentityStateData_.TopROI_.dRoi_h_;
            }
            else
            {
                roi_x = VisualIdentityStateData_.BottomROI_.dRoi_x_;
                roi_y = VisualIdentityStateData_.BottomROI_.dRoi_y_;
                roi_w = VisualIdentityStateData_.BottomROI_.dRoi_w_;
                roi_h = VisualIdentityStateData_.BottomROI_.dRoi_h_;
            }
#ifdef CODE_VERSION_VISUAL
            BoxPoseDetection box_arrange("./config/mask_rcnn",temp_color, temp_depth,
                                         K, distCoeffs, roi_x, roi_y, roi_w, roi_h, VisualIdentityStateData_.fScoreValue_);
            std::queue<std::vector<double>> threeD_pose = box_arrange.process(VisualIdentityStateData_.matSegmentedImage_);

    //        cv::imwrite("out_color_image.png",VisualIdentityStateData_.matSegmentedImage_);
            qDebug() << "threeD_pose size : " << threeD_pose.size();

            while (threeD_pose.size() != 0)
            {
                std::vector<double> temp_pose = threeD_pose.front();
                threeD_pose.pop();

                VisualIdentityStateData_.vecBoxPose_.push_back(Common::Pose(temp_pose[0], temp_pose[1], temp_pose[2]
                        , temp_pose[3], temp_pose[4], temp_pose[5]));
            }
            qDebug() << threeD_pose.size();
#endif

        } catch (std::exception& e)
        {
            qDebug() << __LINE__ << "->" << e.what();
        }

        VisualIdentityStateData_.iwork_mode_ = 1;
    }
    else if (iwork_mode == 2)  // 装货
    {
        VisualIdentityStateData_.vecUpLoadBoxPose_.push_back(Common::Pose(5240, 760, 400, 0,0,0));
        VisualIdentityStateData_.iwork_mode_ = 2;
    }
    else if (iwork_mode == 3)  // 检测偏移
    {
        VisualIdentityStateData_.dValue_ = -0.5;
        VisualIdentityStateData_.iwork_mode_ = 3;
    }
    else if (iwork_mode == 4)  // 像素坐标转换
    {
        rs2::frameset frames = RsPipe_.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::align align_to_color(RS2_STREAM_COLOR);
        auto aligned_frames = align_to_color.process(frames);
        auto depth_frame = aligned_frames.get_depth_frame();
        int cw = color_frame.as<rs2::video_frame>().get_width();
        int ch = color_frame.as<rs2::video_frame>().get_height();
        VisualIdentityStateData_.matOriginalImage_ = cv::Mat(cv::Size(cw,ch), CV_8UC3);
        memcpy(VisualIdentityStateData_.matOriginalImage_.data, (void*)color_frame.get_data(), cw * ch * 3);
        cv::cvtColor(VisualIdentityStateData_.matOriginalImage_, VisualIdentityStateData_.matOriginalImage_, cv::COLOR_BGR2RGB);
        cv::Mat temp_depth(depth_frame.get_height(), depth_frame.get_width(), CV_16UC1, (void*)depth_frame.get_data());
        // 视觉检测
        double fx = 1349.66, fy = 1349.17, cx = 982.689, cy = 545.792;
        Eigen::MatrixXd K(3,3);
        Eigen::VectorXd distCoeffs(5);
        K << fx,0,cx,0,fy,cy,0,0,1;
        distCoeffs << 0,0,0,0,0;
        cv::Mat temp_color = cv::Mat(cv::Size(cw,ch), CV_8UC3);
        memcpy(temp_color.data, (void*)color_frame.get_data(), cw * ch * 3);
        cv::cvtColor(temp_color, temp_color, cv::COLOR_BGR2RGB);
        cv::cvtColor(temp_color, temp_color, cv::COLOR_RGB2BGR);
#ifdef CODE_VERSION_VISUAL
        float roi_x = 0;
        float roi_y = 0;
        float roi_w = 1920;
        float roi_h = 1080;
        BoxPoseDetection box_arrange("./config/mask_rcnn",temp_color, temp_depth,
                                     K, distCoeffs, roi_x, roi_y, roi_w, roi_h, VisualIdentityStateData_.fScoreValue_);
        double rx = 0.0;
        double ry = 0.0;
        double rz = 0.0;
        box_arrange.get_real_value(iQueryU_, iQueryV_, rx, ry, rz);

        VisualIdentityStateData_.dQuX_ = rx;
        VisualIdentityStateData_.dQuY_ = ry;
        VisualIdentityStateData_.dQuZ_ = rz;
        qDebug() << __FILE__ << "  " << __LINE__ << "  " << rx << " " << " " << ry << " " << rz;
#endif
    }


//    pipe.wait_for_frames()
    // 反馈当前状态
    if (pStateCallback_)
    {
        pStateCallback_(VisualIdentityStateData_);
    }
}

void VisualIdentity::CompteContainerOffset()
{
    iwork_mode = 3; // 检测偏移
    WorkBase::SemaphoreActivation();
}

void VisualIdentity::CompteBoxLoadPose()
{
    iwork_mode = 2; // 检测安装位置
    WorkBase::SemaphoreActivation();
}

void VisualIdentity::ActivationIdentification()
{
    // 触发信号量，开启拍照任务
    iwork_mode = 1; // 检测卸载位置
    WorkBase::SemaphoreActivation();
}

void VisualIdentity::ActivationIdentification(int identification_region)
{
    // 触发信号量，开启拍照任务
    iwork_mode = 1;
    iArtificialRegion_ = identification_region;
    WorkBase::SemaphoreActivation();
}

void VisualIdentity::SetROI(int artificial_region,const Common::ROIRectangle &roi_rectangle)
{
    if (artificial_region == 1)
    {
        VisualIdentityStateData_.TopROI_ = roi_rectangle;
    }
    else
    {
        VisualIdentityStateData_.BottomROI_ = roi_rectangle;
    }
    pStateCallback_(VisualIdentityStateData_);
}

void VisualIdentity::SetScoreValue(float score_value)
{
    VisualIdentityStateData_.fScoreValue_ = score_value;
    pStateCallback_(VisualIdentityStateData_);
}

void VisualIdentity::UV2RealPoint(int u, int v)
{
    iwork_mode = 4;
    iQueryU_ = u;
    iQueryV_ = v;
    WorkBase::SemaphoreActivation();
}

void VisualIdentity::EnabledArtificialData(bool data)
{
    bEnabledArtificialData_ = data;
    VisualIdentityStateData_.bArtificialData_ = data;
    pStateCallback_(VisualIdentityStateData_);
}

void VisualIdentity::StartLive(LIVE_CALLBACK live_callbask)
{
    bIsStopLive_ = false;
    pLiveCallback_ = live_callbask;
    thLiveThread_ = std::thread(&VisualIdentity::LiveThread, this);
}

void VisualIdentity::StopLive()
{
    bIsStopLive_ = true;
}

void VisualIdentity::ChangeLiveType(const int& type)
{
    iLiveType_ = type;
}

void VisualIdentity::ControlData(const VisualIdentityControl &data)
{
    if (data.ControlType_ == VisualIdentityControl::ControlType::Init)
    {
        umapArtificialData_[data.iDataId_] = data.vecBoxPoses_;
    }
}

void VisualIdentity::LiveThread()
{
    while (!bIsStopLive_)
    {
        // 获取一帧数据
        try {
            rs2::frameset frames = RsPipe_.wait_for_frames();
            if (iLiveType_ == 0)
            {
                rs2::video_frame color_frame = frames.get_color_frame();
                int cw = color_frame.as<rs2::video_frame>().get_width();
                int ch = color_frame.as<rs2::video_frame>().get_height();
                cv::Mat tempMat(cv::Size(cw,ch), CV_8UC3);
                memcpy(tempMat.data, (void*)color_frame.get_data(), cw * ch * 3);
                cv::cvtColor(tempMat, tempMat, cv::COLOR_BGR2RGB);
                if (pLiveCallback_)
                {
                    pLiveCallback_(tempMat);
                }
            }
            else
            {
                rs2::depth_frame depth_frame = frames.get_depth_frame();
                static rs2::colorizer color_map;
                const int dw = depth_frame.as<rs2::video_frame>().get_width();
                const int dh = depth_frame.as<rs2::video_frame>().get_height();
                cv::Mat tempMat(cv::Size(dw, dh), CV_8UC3);
                memcpy(tempMat.data, (void*)color_map.colorize(depth_frame).get_data(), dw * dh * 3);
                cv::cvtColor(tempMat, tempMat, cv::COLOR_BGR2RGB);
                if (pLiveCallback_)
                {
                    pLiveCallback_(tempMat);
                }
            }

        } catch (std::exception& e)
        {
            qDebug() << e.what();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

}
