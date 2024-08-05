#include "robot_chassis.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <regex>
#include <cmath>
#include <QDebug>

namespace RobotChassis{

RobotChassis::RobotChassis(STATE_CALLBACK call_back)
    : CustomsRobot::WorkBase(CustomsRobot::TIME),
      Log_(LLOG::TLogLevel::DEBUG, "RobotChassis", "./log", 200),
      NDC8TcpClient_(std::bind(&RobotChassis::NDC8_CallBack, this
                        , std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)),
      NMCTcpClient_(std::bind(&RobotChassis::NMC_CallBack, this, std::placeholders::_1))
{
    bExit_ = false;
    pStateCallback_ = call_back;
    WorkBase::SetSleep(100);
    WorkBase::ExecutionThread();
    llSendTime_ = 0;
    ChassisStateData_.ChassisState_ = ChassisState::NetworkNormal;
    ChassisStateData_.NavigationState_ = NavigationState::Idie;
    pStateCallback_(ChassisStateData_);
}

RobotChassis::~RobotChassis()
{
    bExit_ = true;
}

bool RobotChassis::SendGoal(const Common::Pose &pose)
{
    // test  测试时需要更改
//    if (ChassisStateData_.ChassisState_ == ChassisState::NetworkNormal)
//    {
//        ChassisStateData_.NavigationState_ = NavigationState::Underway;
//        llSendTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//        pStateCallback_(ChassisStateData_);
//    }
    // test end
    if (ChassisStateData_.ChassisState_ == ChassisState::NetworkNormal)
    {
        ChassisStateData_.TargetPose_ = pose;
        ChassisStateData_.NavigationState_ = NavigationState::Underway;
        llSendTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        pStateCallback_(ChassisStateData_);
        return true;
    }
    return false;
}

bool RobotChassis::SendGoal(int id)
{
    // test     测试时需要更改
//    if (ChassisStateData_.ChassisState_ == ChassisState::NetworkNormal)
//    {
//        ChassisStateData_.NavigationState_ = NavigationState::Underway;
//        llSendTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//        pStateCallback_(ChassisStateData_);
//    }
    // test end
    if (NDC8TcpClient_.IsConnectServer())
    {
        //<CPI2><Request><Operation Tag="Operation.LocalOrder" Path="Automatic"><GotoPoint PointID="207" /></Operation></Request></CPI2>
        std::stringstream strSendData;
        strSendData << "<CPI2>"
                        << "<Request>"
                            << "<Operation Tag=\"Operation.LocalOrder\" Path=\"Automatic\">"
                                << "<GotoPoint PointID=\""<< id << "\"/>"
                            << "</Operation>"
                        << "</Request>"
                    << "</CPI2>\n";
        // 发送数据到底盘
        if (!NDC8TcpClient_.SendData(strSendData.str().c_str(), (int)strSendData.str().size()))
        {
            std::string error = "NDC8TcpClient_::SendData  error!";

            Log_.AddLog(__LINE__, __FILE__, error.c_str(), error.size(), LLOG::TLogLevel::DEBUG);
        }
        Log_.AddLog(__LINE__, __FILE__, (std::string("send goal : ") + strSendData.str()).c_str(), strSendData.str().size()+std::string("send goal : ").size() , LLOG::TLogLevel::DEBUG);

        ChassisStateData_.NavigationState_ = NavigationState::Underway;
        ChassisStateData_.iTargetStationID_ = id;
        ChassisStateData_.TargetPose_ = umapStationsPoint_[id];
        pStateCallback_(ChassisStateData_);
        return true;
    }
    return false;
}


// 计算补偿后的坐标
void RobotChassis::CalculateOffsetCoordinate(double angle, double distance, double& x, double& y)
{
    double radians = angle * M_PI / 180.0;

   // 更新坐标
   y += distance * std::cos(radians);;
   x += distance * std::sin(radians);
}


double RobotChassis::ConvertAngleChassisToTrueNorth(double angle)
{
    // 将A坐标系角度转换为B坐标系角度
    double angleB = -angle;
    if (angleB < 0.0) {
        angleB += 360.0;
    }
    return angleB;
}


double RobotChassis::ConvertAngleTrueNorthToChassis(double angle)
{
    // 将B坐标系角度转换为A坐标系角度
    double angleA = -angle;
    if (angleA < -180.0) {
        angleA += 360.0;
    }
    return angleA;
}

int RobotChassis::QuantitativeMove(double angle, double distance, double horizontal_offset)
{
    // test     测试时需要更改
//    if (ChassisStateData_.ChassisState_ == ChassisState::NetworkNormal)
//    {
//        ChassisStateData_.NavigationState_ = NavigationState::Underway;
//        llSendTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//        pStateCallback_(ChassisStateData_);
//    }
    // test end
    // 根据当前位置、角度 和 需要前进的距离 算出位置
    // 车辆坐标系 转成 正北坐标
    double cx = -ChassisStateData_.Pose_.y_;
    double cy = ChassisStateData_.Pose_.x_;
    double CurrentAngle = 0.0;
    CurrentAngle = ConvertAngleChassisToTrueNorth(ChassisStateData_.Pose_.yaw_ / (M_PI / 180));

    CalculateOffsetCoordinate(CurrentAngle + angle, distance, cx, cy);
    if (horizontal_offset < 0)
    {
        CalculateOffsetCoordinate(CurrentAngle + -90, std::abs(horizontal_offset), cx, cy);
    }
    else if (horizontal_offset > 0)
    {
        CalculateOffsetCoordinate(CurrentAngle + 90, horizontal_offset, cx, cy);
    }
    // 遍历所有车站点
    /// 获取到最相近的点
    /// 对比点的距离是不是过远
    int iStationID = -1;
    double min_distance = 100000.0;
    for (auto it : umapStationsPoint_)
    {
        double sx = 0.0, sy = 0.0;
        sx = -it.second.y_;
        sy = it.second.x_;
        double SpacingDistance = std::sqrt(std::pow(std::abs(cx - sx), 2) + std::pow(std::abs(cy - sy), 2));

//        if (SpacingDistance < 2000)
//        {
//            qDebug() << "SpacingDistance < 2000   " << it.first  << "  -- " << SpacingDistance;
//        }
        if ((SpacingDistance < 500)                    // 20毫米
                && (SpacingDistance < min_distance) // 算出最小的距离
                && (ChassisStateData_.iCurrentStationID_ != it.first))                            // 忽略当前点
        {
            min_distance = SpacingDistance;
            iStationID = it.first;
        }
    }
    if (iStationID == -1)
    {
        // 没有找到适合的站点
        return -1;
    }
    qDebug() << "前往 " << iStationID << " 号点";
    std::stringstream strLog_data;
    strLog_data << "前往 " << iStationID << " 号点";
    Log_.AddLog(__LINE__, __FILE__, strLog_data.str().c_str(), strLog_data.str().size(), LLOG::TLogLevel::ERROR);
    SendGoal(iStationID);
    return iStationID;
}

// 定量向前
void RobotChassis::QuantitativeAdvance(double distance, double horizontal_offset)
{
    QuantitativeMove(180.0, distance, horizontal_offset);
    ChassisStateData_.NavigationState_ = NavigationState::Underway;

    // test 模拟数据
//    llSendTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // test end
    // 更新状态
    pStateCallback_(ChassisStateData_);

//    if (ChassisStateData_.ChassisState_ == ChassisState::NetworkNormal)
//    {
//        ChassisStateData_.NavigationState_ = NavigationState::Underway;
//        llSendTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//        pStateCallback_(ChassisStateData_);
//    }
}

// 定量后退
void RobotChassis::QuantitativeReceding(double distance, double horizontal_offset)
{
    QuantitativeMove(0.0, distance, horizontal_offset);
    ChassisStateData_.NavigationState_ = NavigationState::Underway;

    // test 模拟数据
//    llSendTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // test end
    // 更新状态
    pStateCallback_(ChassisStateData_);

//    if (ChassisStateData_.ChassisState_ == ChassisState::NetworkNormal)
//    {
//        ChassisStateData_.NavigationState_ = NavigationState::Underway;
//        llSendTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//        pStateCallback_(ChassisStateData_);
//    }
}

bool isNumeric(const std::string& str) {
    try {
        std::stoi(str);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool parseData(const std::string& input, std::vector<int> &data) {
//    std::vector<int> result;
    std::istringstream iss(input);
    std::string token;
//    std::regex pattern("^\\d+$");
    while (std::getline(iss, token, ','))
    {
        try {
            if (!isNumeric(token))
            {
                continue;
            }
            int value = std::stoi(token);
            data.push_back(value);
        }
        catch (const std::exception& e) {
            // 忽略解析错误
        }
    }
    if (data.size() == 0)
    {
        return false;
    }
    return true;
}

// 来自界面的交互数据
void RobotChassis::ControlData(const ChassisControl& control_data)
{
    switch (control_data.ControlType_)
    {
        case ChassisControl::ControlType::Init:
        {
            // 打开文件
            std::ifstream fin;
            fin.open(control_data.strValue_.c_str(), std::ios::in);
            if (!fin.is_open())
            {
                std::string log_data = "Unable to open file ：" + control_data.strValue_;
                Log_.AddLog(__LINE__, __FILE__, log_data.c_str(), log_data.size(), LLOG::TLogLevel::ERROR);
                break;
            }
            std::string buff;
            while (std::getline(fin, buff)) // 读一行
            {
                std::vector<int> vecData;
                if (parseData(buff, vecData))
                {
                    Common::Pose StationPose(vecData[1], vecData[2], 0.0
                            , 0.0, 0.0, (vecData[3] / 100) * M_PI / 180.0); // 转成弧度
                    umapStationsPoint_[vecData[0]] = StationPose;
                }
            }
            fin.close();
            break;
        }
        case ChassisControl::ControlType::NMC_Init:
        {
            if (!NMCTcpClient_.IsConnectServer())
            {
                std::string ip = control_data.strValue_;
                NMCTcpClient_.SetServerIP(ip);
                NMCTcpClient_.SetServerPort(control_data.iValue_);
                NMCTcpClient_.ConnectServer();
            }
            break;
        }
        case ChassisControl::ControlType::NDC8_Init:
        {
            if (!NDC8TcpClient_.IsConnectServer())
            {
                std::string ip = control_data.strValue_;
                NDC8TcpClient_.SetServerIP(ip);
                NDC8TcpClient_.SetServerPort(control_data.iValue_);
                NDC8TcpClient_.ConnectServer();
            }
            break;
        }
        case ChassisControl::ControlType::On_Forward:   // 前
        {
            ControlForward();
            break;
        }
        case ChassisControl::ControlType::On_Backwards: // 后
        {
            ControlBackwards();
            break;
        }
        case ChassisControl::ControlType::On_Left:  // 左
        {
            ControlLeft();
            break;
        }
        case ChassisControl::ControlType::On_Right: // 右
        {
            ControlRight();
            break;
        }
        case ChassisControl::ControlType::On_Stop:  // 停
        {
            ControlStop();
            break;
        }
        case ChassisControl::ControlType::On_Turn: // 旋转
        {
            ControlTurn(control_data.dValue_);
            break;
        }
        case ChassisControl::ControlType::Get_Lidar: // 旋转
        {
            GetLidar(0);
            break;
        }
        case ChassisControl::ControlType::Send_Goal: // 旋转
        {
            SendGoal(control_data.iValue_);
            break;
        }
        default :
        {
            break;
        }
    }
}

// 底盘返回的数据返回
void RobotChassis::TcpClientCallBack(const ChassisStateData& data)
{
    (void)data;
    qDebug() << "RobotChassis::TcpClientCallBack";
}


// 底盘返回数据的回调函数
void RobotChassis::NDC8_CallBack(const std::string &attr_name, const std::string &attr_value, const std::string &value)
{
    bool bUpdata = false;
//    if (value == "")
//    {
//        qDebug() << attr_name.c_str() << "---" << attr_value.c_str();
//    }
//    else
//    {
//        qDebug() << attr_name.c_str() << "---" << attr_value.c_str() << " -> " << value.c_str();
//    }

    if (attr_name == "Tag")
    {
        if (attr_value == "Position.X")
        {
            ChassisStateData_.Pose_.x_ = std::stod(value.c_str());
            bUpdata = true;
        }
        else if (attr_value == "Position.Y")
        {
            ChassisStateData_.Pose_.y_ = std::stod(value.c_str());
            bUpdata = true;
        }
        else if (attr_value == "Position.Angle")
        {
//            ChassisStateData_.Pose_.z_ = std::stod(value.c_str());
            ChassisStateData_.Pose_.SetPose(ChassisStateData_.Pose_.x_
                                            , ChassisStateData_.Pose_.y_
                                            , 0.0
                                            , 0.0
                                            , 0.0
                                            , std::stod(value.c_str()) * (M_PI / 180.0)); // 需要转成弧度
        }
        else if (attr_value == "AGV_1_24.soc") // 电量
        {
            ChassisStateData_.dVoltage_ = std::stod(value.c_str());
            qDebug() << "电量 ： " << ChassisStateData_.dVoltage_;
            bUpdata = true;
        }
        else if (attr_value == "UserDefined.StopWord") // 停止状态
        {
            int iValue = std::stoi(value.c_str());
            qDebug() << "停止状态 ： " << std::stod(value.c_str());
            std::list<int> ValidValue;
            Common::BinaryValidIndex(iValue, ValidValue);
            for (auto it : ValidValue)
            {
                qDebug() << "Valid Index : " << it;
            }
//            qDebug() << "停止状态 ： " << std::stod(value.c_str());
//            bUpdata = true;
        }
        else if (attr_value == "LayoutPosition.Point") // 当前到达点
        {
            int StationID = std::stod(value.c_str());
            if (StationID != 0)
            {
                if (ChassisStateData_.iCurrentStationID_ != StationID)
                {
                    ChassisStateData_.iCurrentStationID_ = StationID;
                    qDebug() << u8"当前站点:" << StationID;
//                    ChassisStateData_.Pose_ = umapStationsPoint_[ChassisStateData_.iCurrentStationID_];
                    bUpdata = true;
                }

                if (ChassisStateData_.NavigationState_ == NavigationState::Underway)
                {
                    if (ChassisStateData_.iTargetStationID_ == ChassisStateData_.iCurrentStationID_)
                    {
                        ChassisStateData_.NavigationState_ = NavigationState::Arrive;
                        ChassisStateData_.iTargetStationID_ = 0;
                        bUpdata = true;
                    }
                }
            }
        }
    }
    if (bUpdata)
    {
        pStateCallback_(ChassisStateData_);
    }
}


void RobotChassis::NMC_CallBack(const std::string &data)
{
    const unsigned char* pData = (const unsigned char*)data.c_str();
    std::vector<Common::Pose> points;

    switch (pData[0])
    {
        case 0x73:  //  雷达数据
        {
            // 写到文件
            std::string log_data;
            std::stringstream ioss;
            for (int j = 0; j < (int)data.size(); j++)
            {
                ioss << std::hex << std::setw(2) << std::setfill('0') << (unsigned int)data.c_str()[j];
                log_data.append(ioss.str()).append(" ");
                ioss.str("");
            }
            Log_.AddLog(__LINE__, __FILE__, log_data.c_str(), log_data.size(), LLOG::TLogLevel::DEBUG);
            //

            uint16_t max_leng = 0;
            uint16_t data_leng = pData[3];
            data_leng <<= 8;
            data_leng |= 0x00FF;
            data_leng |= pData[2];
            double angle = -90.0;

            for (int i = 0; i < (int)data_leng; i+=2)
            {
                if (angle > 90.0)
                {
                    break;
                }
                uint16_t line_leng = pData[i+8+1];
                line_leng <<= 8;
                line_leng |= pData[i+8];

                if (max_leng < line_leng)
                {
                    max_leng = line_leng;
                }

                double x, y;
                CalculateRelativePosition(angle, (double)line_leng, x, y);
                points.push_back(Common::Pose(x, y, 0.0, 0.0, 0.0, 0.0));
                angle += 0.5;
//                if (x < 1 && x > -1)
//                {
//                    qDebug() << x;
//                }
//                if (y < 1 && y > -1)
//                {
//                    qDebug() << y;
//                }
            }
            // 将图刷新到 交互界面
//            VisualIdentityStateData_.matSegmentedImage_ = cv::Mat(cv::Size(dw,dh), CV_8UC3);
//            memcpy(VisualIdentityStateData_.matSegmentedImage_.data, (void*)color_map.colorize(depth_frame).get_data(), dw * dh * 3);
            ChassisStateData_.matLidarImage_.release();
            ChassisStateData_.matLidarImage_ = cv::Mat(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));
//            cv::Mat image(max_leng / 10, max_leng / 10, CV_8UC3, cv::Scalar(0, 0, 0));
            cv::line(ChassisStateData_.matLidarImage_, cv::Point(300, 0)
                     , cv::Point(300, 600)
                     , cv::Scalar(110, 110, 170), 1);

            cv::line(ChassisStateData_.matLidarImage_, cv::Point(0, 450)
                     , cv::Point(ChassisStateData_.matLidarImage_.cols, 450)
                     , cv::Scalar(110, 110, 170), 1);
            for (auto itpoint : points)
            {
                 // 选择点的位置
                 cv::Point center((itpoint.x_ / 10) + 300, 450 - (itpoint.y_ / 10));
                 // 设置点的颜色
                 cv::Scalar color(255, 255, 255);
                 // 在图像上画点
                 cv::circle(ChassisStateData_.matLidarImage_, center, 1, color, -1);
            }
            // 更新数据
            pStateCallback_(ChassisStateData_);
            // -------------测试用 保存一张点云图片------------------
//            cv::Mat image(6000, 6000, CV_8UC3, cv::Scalar(255, 255, 255));
//            // 画第一条竖直线
//            cv::line(image, cv::Point(image.cols / 2, 0), cv::Point(image.cols / 2, image.rows), cv::Scalar(0, 0, 0), 2);

//            // 画第二条水平线
//            cv::line(image, cv::Point(0, image.rows / 2), cv::Point(image.cols, image.rows / 2), cv::Scalar(0, 0, 0), 2);
//            for (auto itpoint : points)
//            {
//                 // 选择点的位置
//                 cv::Point center(itpoint.x_ + 3000, 3000 - itpoint.y_);

//                 // 设置点的颜色
//                 cv::Scalar color(115, 130, 40);

//                 // 在图像上画点
//                 cv::circle(image, center, 5, color, -1); // 半径为5的实心点
//            }
//             cv::imwrite("test.jpg", image);
            // -------------测试用 保存一张点云图片-----结束-------------
        }
        default :
        {
            break;
        }
    }
}

void RobotChassis::CalculateRelativePosition(double angle, double length, double& x, double& y)
{
    // 将角度转换为弧度points
    double radians = angle * M_PI / 180;

    // 计算相对坐标位置
    y = length * cos(radians);
    x = length * sin(radians);
}

// 工作线程
void RobotChassis::WorkFun()
{
    // test  测试代码，等待一段时间 模仿车辆正在运行
//    if ((ChassisStateData_.NavigationState_ == NavigationState::Underway) \
//        && (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - llSendTime_ > 5000))
//    {
//        // test 到达
//        llSendTime_ = 0;
//        ChassisStateData_.NavigationState_ = NavigationState::Arrive;
//        pStateCallback_(ChassisStateData_);
//        return;
//    }
    // test end


    // 检查是否与对应的服务器建立连接
    if (!NDC8TcpClient_.IsConnectServer() && !bExit_)
    {
//        NDC8TcpClient_.ConnectServer(); // 尝试连接服务器
        if (NDC8TcpClient_.Start())
        {
            std::stringstream strSendData;
            strSendData << "<CPI2>"
                        << "<Request>"
                            << "<Subscribe Tag=\"us_1\" MinInterval=\"" << 500 << "\">"
                                << "<Item Tag=\"Position.X\" Path=\"Status.Position.X\" />"
                                << "<Item Tag=\"Position.Y\" Path=\"Status.Position.Y\" />"
                                << "<Item Tag=\"Position.Angle\" Path=\"Status.Position.Angle\"/>"
                                << "<Item Tag=\"UserDefined.StopWord\" Path=\"Status.UserDefined.StopWord\"/>"
                                << "<Item Tag=\"AGV_1_24.soc\" Path=\"Status.AGV_1_24.soc\"/>"
                                << "<Item Tag=\"LayoutPosition.Point\" Path=\"Status.LayoutPosition.Point\"/>"
                                << "<Item Tag=\"LayoutPosition.Segment\" Path=\"Status.LayoutPosition.Segment\"/>"
                                << "<Item Tag=\"LayoutPosition.Distance\" Path=\"Status.LayoutPosition.Distance\"/>"
                                << "<Item Tag=\"LayoutPosition.RemainingDistance\" Path=\"Status.LayoutPosition.RemainingDistance\"/>"
                            << "</Subscribe>"
                        << "</Request>"
                        << "</CPI2>\n";
            // 发送数据到底盘
            if (!NDC8TcpClient_.SendData(strSendData.str().c_str(), (int)strSendData.str().size()))
            {
                std::string error = "NDC8TcpClient_::SendInitData  error!";

                Log_.AddLog(__LINE__, __FILE__, error.c_str(), error.size(), LLOG::TLogLevel::DEBUG);
            }
        }
    }

    if (!NMCTcpClient_.IsConnectServer() && !bExit_)
    {
//        NMCTcpClient_.ConnectServer(); // 尝试连接服务器
        NMCTcpClient_.Start();
    }

    // 如果这帧数据是完成的，那么就把该状态换回空闲
    if (ChassisStateData_.NavigationState_ == NavigationState::Arrive)
    {
        ChassisStateData_.NavigationState_ = NavigationState::Idie;
        ChassisStateData_.TargetPose_.SetZero();
        pStateCallback_(ChassisStateData_);
    }
}

void RobotChassis::Pause()
{
    // 停止底盘运动
    ControlStop();
    PauseBackupState_ = ChassisStateData_.NavigationState_;
    if (ChassisStateData_.NavigationState_ == NavigationState::Underway)
    {   // 记录当前 需要前往的 目标点

        PauseBackupTarget_ = ChassisStateData_.iTargetStationID_;
    }
    ChassisStateData_.NavigationState_ = NavigationState::Pause;

    pStateCallback_(ChassisStateData_);
}

// 前进
bool RobotChassis::ControlForward()
{
    ControlVelocity_.dLinearvelocity_ = ControlVelocity_.dLinearvelocity_ > 10
            ? ControlVelocity_ .dLinearvelocity_
            : ControlVelocity_.dLinearvelocity_ + 0.5;

    if (NDC8TcpClient_.IsConnectServer())
    {
        std::stringstream strSendData;
        // NDC8.Manual.PlcSpeed
        strSendData << "<CPI2>"
                        << "<Request>"
                            << "<Set Tag=\"Manual\">"
                                << "<Item Tag=\"Manual.PlcSpeed\" Path=\"Status.Manual.PlcSpeed\">"
                                    << ControlVelocity_.dLinearvelocity_
                                << "</Item>"
                            << "</Set>"
                        << "</Request>"
                    << "</CPI2>";
        // 发送数据到底盘
        if (!NDC8TcpClient_.SendData(strSendData.str().c_str(), (int)strSendData.str().size()))
        {
            std::string error = "NDC8TcpClient_::SendData  error!";

            Log_.AddLog(__LINE__, __FILE__, error.c_str(), error.size(), LLOG::TLogLevel::DEBUG);
        }
        Log_.AddLog(__LINE__, __FILE__, (std::string("send control forward : ") + strSendData.str()).c_str(), strSendData.str().size(), LLOG::TLogLevel::DEBUG);
        return true;
    }
    return false;
}

// 后退
bool RobotChassis::ControlBackwards()
{
    ControlVelocity_.dLinearvelocity_ = ControlVelocity_.dLinearvelocity_ > 10
            ? ControlVelocity_ .dLinearvelocity_
            : ControlVelocity_.dLinearvelocity_ - 0.5;

    if (NDC8TcpClient_.IsConnectServer())
    {
        std::stringstream strSendData;
        // NDC8.Manual.PlcSpeed
        strSendData << "<CPI2>"
                        << "<Request>"
                            << "<Set Tag=\"Manual\">"
                                << "<Item Tag=\"Manual.PlcSpeed\" Path=\"Status.Manual.PlcSpeed\">"
                                    << ControlVelocity_.dLinearvelocity_
                                << "</Item>"
                            << "</Set>"
                        << "</Request>"
                    << "</CPI2>";
        // 发送数据到底盘
        if (!NDC8TcpClient_.SendData(strSendData.str().c_str(), (int)strSendData.str().size()))
        {
            std::string error = "NDC8TcpClient_::SendData  error!";

            Log_.AddLog(__LINE__, __FILE__, error.c_str(), error.size(), LLOG::TLogLevel::DEBUG);
        }
        Log_.AddLog(__LINE__, __FILE__, (std::string("send control backwards : ") + strSendData.str()).c_str(), strSendData.str().size(), LLOG::TLogLevel::DEBUG);
        return true;
    }
    return false;
}

// 向左
bool RobotChassis::ControlLeft()
{
    return false;
}

// 向右
bool RobotChassis::ControlRight()
{
    return false;
}

// 停止
bool RobotChassis::ControlStop()
{
    ControlVelocity_.SetZero();

    if (NDC8TcpClient_.IsConnectServer())
    {
        //UserDefined.RobotState
        std::stringstream strSendData;
        strSendData << "<CPI2>"
                        << "<Request>"
                            << "<Set Tag=\"UserDefined\">"
                                << "<Item Tag=\"UserDefined.RobotState\" Path=\"Status.UserDefined.RobotState\">"
                                    << 1    // 0：无故障，1：停止
                                << "</Item>"
                            << "</Set>"
                        << "</Request>"
                    << "</CPI2>";
        // 发送数据到底盘
        if (!NDC8TcpClient_.SendData(strSendData.str().c_str(), (int)strSendData.str().size()))
        {
            std::string error = "NDC8TcpClient_::SendData  error!";

            Log_.AddLog(__LINE__, __FILE__, error.c_str(), error.size(), LLOG::TLogLevel::DEBUG);
        }
        Log_.AddLog(__LINE__, __FILE__, (std::string("send control stop : ") + strSendData.str()).c_str(), strSendData.str().size(), LLOG::TLogLevel::DEBUG);
        return true;
    }
    return false;
}

// 旋转
bool RobotChassis::ControlTurn(double angle)
{
    if (NDC8TcpClient_.IsConnectServer())
    {
        std::stringstream strSendData;
        // NDC8.Manual.PlcAngle
        strSendData << "<CPI2>"
                        << "<Request>"
                            << "<Set Tag=\"Manual\">"
                                << "<Item Tag=\"Manual.PlcAngle\" Path=\"Status.Manual.PlcAngle\">"
                                    << angle
                                << "</Item>"
                            << "</Set>"
                        << "</Request>"
                    << "</CPI2>";
        // 发送数据到底盘
        if (!NDC8TcpClient_.SendData(strSendData.str().c_str(), (int)strSendData.str().size()))
        {
            std::string error = "NDC8TcpClient_::SendData  error!";

            Log_.AddLog(__LINE__, __FILE__, error.c_str(), error.size(), LLOG::TLogLevel::DEBUG);
        }
        Log_.AddLog(__LINE__, __FILE__, (std::string("send control turn : ") + strSendData.str()).c_str(), strSendData.str().size(), LLOG::TLogLevel::DEBUG);
        return true;
    }
    return false;
}

bool RobotChassis::GetLidar(int id)
{
    // 72 00 08 00 00 00 00 00
    unsigned char data[8];
    memset(data, 0, 8);
    data[0] = 0x72;
    data[2] = 0x08;
    uint16_t temp = (uint16_t)id;

    data[4] = temp & 0x0011;
    data[5] = temp >> 8;

    NMCTcpClient_.SendData((char*)data, 8);

//    std::stringstream ss;
//    ss << std::hex << std::setw(8) << std::setfill('0');
//    for (int i = 0; i < 8; i++)
//    {
//        ss << data[i] << " ";
//    }
//
/*    unsigned char test_data[] = { 0x73, 0x00, 0xda, 0x02, 0x00, 0x00, 0x00, 0x00, 0x83, 0x04, 0x83, 0x04, 0x82, 0x04, 0x83, 0x04, 0x83, 0x04, 0x81, 0x04 \
    , 0x80, 0x04, 0x86, 0x04, 0x87, 0x04, 0x87, 0x04, 0x89, 0x04, 0x89, 0x04, 0x89, 0x04, 0x8d, 0x04, 0x8d, 0x04, 0x8e, 0x04 \
    , 0x90, 0x04, 0x90, 0x04, 0x92, 0x04, 0x95, 0x04, 0x97, 0x04, 0x9a, 0x04, 0x9c, 0x04, 0x86, 0x04, 0x38, 0x04, 0x38, 0x04 \
    , 0x09, 0x04, 0xe4, 0x03, 0xc6, 0x03, 0x9b, 0x04, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0x15, 0x03, 0xdf, 0x03 \
    , 0xad, 0x05, 0xa9, 0x02, 0x95, 0x02, 0x7e, 0x02, 0x6c, 0x02, 0x6c, 0x02, 0x6c, 0x02, 0x6c, 0x02, 0x70, 0x02, 0x73, 0x02 \
    , 0x76, 0x02, 0x78, 0x02, 0x7b, 0x02, 0x7b, 0x02, 0x7e, 0x02, 0x81, 0x02, 0x86, 0x02, 0x88, 0x02, 0x8e, 0x02, 0x20, 0x05 \
    , 0xb0, 0x03, 0xb0, 0x03, 0xc4, 0x09, 0x93, 0x03, 0x4c, 0x05, 0x55, 0x05, 0x60, 0x05, 0x64, 0x05, 0x6f, 0x05, 0x6f, 0x05 \
    , 0x7b, 0x05, 0x81, 0x05, 0x8a, 0x05, 0xea, 0x05, 0xd5, 0x05, 0xbe, 0x05, 0xaf, 0x05, 0xaf, 0x05, 0xa0, 0x05, 0x90, 0x05 \
    , 0xc4, 0x09, 0xc4, 0x09, 0x16, 0x04, 0x4b, 0x05, 0x3a, 0x05, 0x3a, 0x05, 0xf9, 0x03, 0xae, 0x03, 0xa0, 0x03, 0xa5, 0x03 \
    , 0xab, 0x03, 0xa1, 0x03, 0x9c, 0x03, 0x9c, 0x03, 0x9d, 0x03, 0xa8, 0x03, 0xaf, 0x03, 0xc4, 0x09, 0xc4, 0x09, 0xd2, 0x03 \
    , 0xdf, 0x03, 0xdf, 0x03, 0xe4, 0x03, 0x51, 0x09, 0x6e, 0x09, 0xc4, 0x09, 0x73, 0x09, 0x69, 0x09, 0xbb, 0x07, 0xbb, 0x07 \
    , 0x83, 0x09, 0xe5, 0x07, 0xc4, 0x09, 0x12, 0x08, 0xa3, 0x0a, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xec, 0x04 \
    , 0xdb, 0x04, 0xd7, 0x04, 0xce, 0x04, 0xc7, 0x04, 0xc0, 0x04, 0xc0, 0x04, 0xbe, 0x04, 0xd5, 0x04, 0xef, 0x04, 0x02, 0x05 \
    , 0x88, 0x08, 0x80, 0x08, 0x75, 0x08, 0x75, 0x08, 0x69, 0x08, 0x60, 0x08, 0x57, 0x08, 0x4e, 0x08, 0x37, 0x03, 0x32, 0x03 \
    , 0x3a, 0x03, 0x3a, 0x03, 0xc4, 0x09, 0x25, 0x08, 0xc4, 0x09, 0xc4, 0x09, 0x21, 0x03, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09 \
    , 0xc4, 0x09, 0xf7, 0x07, 0xc4, 0x09, 0xc4, 0x09, 0xe7, 0x07, 0xc4, 0x09, 0xdd, 0x07, 0xdd, 0x07, 0xc4, 0x09, 0xd1, 0x07 \
    , 0x94, 0x02, 0xa2, 0x02, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09 \
    , 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xb0, 0x07, 0xae, 0x07, 0xab, 0x07, 0xab, 0x07, 0xac, 0x07, 0xaa, 0x07 \
    , 0xa8, 0x07, 0xa8, 0x07, 0xa8, 0x07, 0xa9, 0x07, 0xa8, 0x07, 0xaa, 0x07, 0xa8, 0x07, 0xab, 0x07, 0xc4, 0x09, 0xc4, 0x09 \
    , 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09 \
    , 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xa3, 0x02, 0x99, 0x02, 0x99, 0x02, 0xd4, 0x07, 0x52, 0x04, 0x46, 0x04, 0x47, 0x04 \
    , 0x4a, 0x04, 0x4c, 0x04, 0x50, 0x04, 0x50, 0x04, 0x53, 0x04, 0x56, 0x04, 0x5a, 0x04, 0x09, 0x08, 0x0e, 0x08, 0x15, 0x08 \
    , 0xf0, 0x05, 0xf0, 0x05, 0x23, 0x08, 0x2c, 0x08, 0x36, 0x08, 0xf9, 0x05, 0xba, 0x05, 0x4b, 0x08, 0x56, 0x08, 0x56, 0x08 \
    , 0x60, 0x08, 0x6d, 0x08, 0x73, 0x08, 0xd2, 0x04, 0x87, 0x08, 0x45, 0x03, 0x9d, 0x08, 0x9d, 0x08, 0xa9, 0x08, 0xb5, 0x08 \
    , 0x25, 0x04, 0xcd, 0x08, 0xda, 0x08, 0x5c, 0x03, 0xa9, 0x08, 0xa9, 0x08, 0xb6, 0x03, 0x77, 0x08, 0xbc, 0x08, 0xc6, 0x08 \
    , 0x78, 0x03, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0xc4, 0x09, 0x6d, 0x0d, 0x96, 0x03, 0x99, 0x09, 0x79, 0x09, 0x84, 0x01 \
    , 0x85, 0x01, 0x85, 0x01, 0x81, 0x01, 0x7c, 0x01, 0x77, 0x01, 0x72, 0x01, 0x6e, 0x01, 0x69, 0x01, 0x66, 0x01, 0x66, 0x01 \
    , 0x62, 0x01, 0x5d, 0x01, 0x5b, 0x01, 0x59, 0x01, 0x55, 0x01, 0x51, 0x01, 0x4f, 0x01, 0x4f, 0x01, 0x4e, 0x01, 0x52, 0x01 \
    , 0x55, 0x01, 0x58, 0x01, 0x5d, 0x01, 0x61, 0x01, 0x66, 0x01, 0x66, 0x01, 0x6a, 0x01, 0x6f, 0x01, 0x72, 0x01, 0x7a, 0x01 \
    , 0x7d, 0x01, 0x85, 0x01, 0x8b, 0x01, 0x8b, 0x01, 0xd6, 0x04, 0xcc, 0x04, 0xc1, 0x04, 0xba, 0x04, 0xb1, 0x04, 0xa9, 0x04 \
    , 0xa2, 0x04, 0xa2, 0x04, 0xaa, 0x04, 0xbc, 0x04, 0xcf, 0x04, 0xe0, 0x04, 0xf6, 0x04, 0x79, 0x06, 0x6e, 0x06, 0x6e, 0x06 \
    , 0x1b, 0x06, 0x70, 0x06, 0x52, 0x06, 0x68, 0x06, 0xc4, 0x09, 0x4a, 0x06, 0x52, 0x06, 0x52, 0x06, 0xe7, 0x05, 0x39, 0x06 \
    , 0x37, 0x06, 0x3b, 0x06, 0x32, 0x06, 0x1e, 0x06, 0x26, 0x06, 0x26, 0x06, 0x22, 0x06, 0x16, 0x06, 0x0a, 0x06, 0x13, 0x06 \
    , 0xc4, 0x05, 0xfe, 0x05, 0xa0, 0x04, 0xa0, 0x04, 0x9f, 0x04, 0x9b, 0x04, 0x99, 0x04, 0x95, 0x04, 0x93, 0x04, 0x91, 0x04 \
    , 0x92, 0x04, 0x92, 0x04, 0xc4, 0x09, 0xea, 0x05, 0x89, 0x05, 0xcd, 0x05, 0xc4, 0x09, 0xc4, 0x09, 0x35, 0x05, 0x35, 0x05 \
    , 0x29, 0x05, 0x47, 0x05, 0x48, 0x05, 0x29, 0x05, 0x28, 0x05, 0x28, 0x05, 0x24, 0x05, 0x24, 0x05, 0x25, 0x05, 0x22, 0x05 \
    , 0x2a, 0x05, 0x75, 0x05, 0x2c, 0x08, 0x30, 0x08, 0x2f, 0x08};
    ////    unsigned char test_data[] = {0x73, 0x00, 0xDA, 0x02, 0x00, 0x00};
    std::string temp_data((char*)test_data, 730);
    NMC_CallBack(temp_data);*/
    return false;
}

}
