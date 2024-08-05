#include "laser_ranging_jrt_bb2x.h"
#include <QDebug>
namespace RobotArm{

LaserRangingJrtBb2x::LaserRangingJrtBb2x(std::string device_name, int baud_rate)
    : Device_(device_name, baud_rate)
{
    uiDistance_ = 0;
    bStop_ = false;
    llLastDataTime_ = 0;
    thUpdata_ = std::thread(&LaserRangingJrtBb2x::ThreadFun, this);
}

LaserRangingJrtBb2x::~LaserRangingJrtBb2x()
{
    bStop_ = true;
    Device_.StopCommunication();
    if (thUpdata_.joinable())
    {
        thUpdata_.join();
    }
    llLastDataTime_ = 0;
    uiDistance_ = 0;
}

// 开启设备
bool LaserRangingJrtBb2x::OpenDevice()
{
    if (!Device_.IsCommunication())
    {
        return false;
    }
    // AA 00 01 BE 00 01 00 01 C1
    char send_data[] = {static_cast<char>(0xAA), static_cast<char>(0x00), static_cast<char>(0x01)
                        , static_cast<char>(0xBE), static_cast<char>(0x00), static_cast<char>(0x01)
                        , static_cast<char>(0x00), static_cast<char>(0x01), static_cast<char>(0xC1)};
    Device_.WriteData(std::string(send_data, 9));
    return true;
}

// 关闭设备
bool LaserRangingJrtBb2x::CloseDevice()
{
    if (!Device_.IsCommunication())
    {
        return false;
    }
    // AA 00 01 BE 00 01 00 00 C0
    char send_data[] = {static_cast<char>(0xAA), static_cast<char>(0x00), static_cast<char>(0x01)
                        , static_cast<char>(0xBE), static_cast<char>(0x00), static_cast<char>(0x01)
                        , static_cast<char>(0x00), static_cast<char>(0x00), static_cast<char>(0xC0)};
    Device_.WriteData(std::string(send_data, 9));
    return true;
}

// 停止测量
bool LaserRangingJrtBb2x::StopMeasurement()
{
    if (!Device_.IsCommunication())
    {
        return false;
    }
    // 58
    Device_.WriteData(std::string(static_cast<char>(0x58), 1));
    return true;
}

//获取一次自动测量
bool LaserRangingJrtBb2x::GetAnAutoMeasurement()
{
    if (!Device_.IsCommunication())
    {
        return false;
    }
    // 0xAA 0x00 0x00 0x20 0x00 0x01 0x00 0x00 0x21
    char send_data[] = {static_cast<char>(0xAA), static_cast<char>(0x00), static_cast<char>(0x00)
                        , static_cast<char>(0x20), static_cast<char>(0x00), static_cast<char>(0x01)
                        , static_cast<char>(0x00), static_cast<char>(0x00), static_cast<char>(0x21)};
    Device_.WriteData(std::string(send_data, 9));
    return true;
}

// 获取一次慢速测量
bool LaserRangingJrtBb2x::GetAnSlowMeasurement()
{
    if (!Device_.IsCommunication())
    {
        return false;
    }
    // 0xAA 0x00 0x00 0x20 0x00 0x01 0x00 0x01 0x22
    char send_data[] = {static_cast<char>(0xAA), static_cast<char>(0x00), static_cast<char>(0x00)
                        , static_cast<char>(0x20), static_cast<char>(0x00), static_cast<char>(0x01)
                        , static_cast<char>(0x00), static_cast<char>(0x01), static_cast<char>(0x22)};
    Device_.WriteData(std::string(send_data, 9));
    return true;
}

// 获取一次快速测量
bool LaserRangingJrtBb2x::GetAnFastMeasurement()
{
    if (!Device_.IsCommunication())
    {
        return false;
    }
    // 0xAA 0x00 0x00 0x20 0x00 0x01 0x00 0x02 0x23
    char send_data[] = {static_cast<char>(0xAA), static_cast<char>(0x00), static_cast<char>(0x00)
                        , static_cast<char>(0x20), static_cast<char>(0x00), static_cast<char>(0x01)
                        , static_cast<char>(0x00), static_cast<char>(0x02), static_cast<char>(0x23)};
    Device_.WriteData(std::string(send_data, 9));
    return true;
}

//连续获取自动测量
bool LaserRangingJrtBb2x::ContinuousAcquisitionAutoMeasurement()
{
    if (!Device_.IsCommunication())
    {
        return false;
    }
    // 0xAA 0x00 0x00 0x20 0x00 0x01 0x00 0x04 0x25
    char send_data[] = {static_cast<char>(0xAA), static_cast<char>(0x00), static_cast<char>(0x00)
                        , static_cast<char>(0x20), static_cast<char>(0x00), static_cast<char>(0x01)
                        , static_cast<char>(0x00), static_cast<char>(0x04), static_cast<char>(0x25)};
    Device_.WriteData(std::string(send_data, 9));
    return true;
}

//连续获取慢速测量
bool LaserRangingJrtBb2x::ContinuousAcquisitionSlowMeasurement()
{
    if (!Device_.IsCommunication())
    {
        return false;
    }
    // 0xAA 0x00 0x00 0x20 0x00 0x01 0x00 0x05 0x26
    char send_data[] = {static_cast<char>(0xAA), static_cast<char>(0x00), static_cast<char>(0x00)
                        , static_cast<char>(0x20), static_cast<char>(0x00), static_cast<char>(0x01)
                        , static_cast<char>(0x00), static_cast<char>(0x05), static_cast<char>(0x26)};
    Device_.WriteData(std::string(send_data, 9));
    return true;
}

//连续获取快速测量
bool LaserRangingJrtBb2x::ContinuousAcquisitionFastMeasurement()
{
    if (!Device_.IsCommunication())
    {
        return false;
    }
    // 0xAA 0x00 0x00 0x20 0x00 0x01 0x02 0x06 0x2D
    char send_data[] = {static_cast<char>(0xAA), static_cast<char>(0x00), static_cast<char>(0x00)
                        , static_cast<char>(0x24), static_cast<char>(0x00), static_cast<char>(0x01)
                        , static_cast<char>(0x02), static_cast<char>(0x06), static_cast<char>(0x2D)};
    Device_.WriteData(std::string(send_data, 9));
    return true;
}


uint32_t LaserRangingJrtBb2x::GetDistance()
{
    return uiDistance_;
}


void LaserRangingJrtBb2x::ThreadFun()
{
    while(!bStop_)
    {
        if (!Device_.IsCommunication())
        {
            if (!Device_.EstablishCommunication())
            {
                // 连接失败，过一会重连
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count()
            - llLastDataTime_ > 1000)   // 超过一秒没有收到最新的数据
        {
            ContinuousAcquisitionAutoMeasurement();
            llLastDataTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count();
        }
        // 已经连接成功
        std::string get_data;
        Device_.ReadData(get_data);
        strBufferData_ += get_data;
        while (strBufferData_.size() > 4)   // 需要识别消息类型标签 所以至少4个字节
        {
            int head_index = -1;
            // 距离返回消息
            if ((head_index = strBufferData_.find(0xAA)) > -1)
            {
                if (strBufferData_[head_index+2] == 0x00
                    && strBufferData_[head_index+3] == 0x22)
                {   // 判断长度对不对
                    if((int)strBufferData_.size() < head_index + 13) // 该消息有13个字节
                    {
                        break;
                    }
                    // 获取长度
                    uint32_t tempDistance = 0;
                    tempDistance = (uint32_t)( ((strBufferData_[head_index + 6] & 0xFF) << 8)
                                            | ((strBufferData_[head_index + 7] & 0xFF) << 8)
                                            | ((strBufferData_[head_index + 8] & 0xFF) << 8)
                                            | (strBufferData_[head_index + 9] & 0xFF) );
                    uiDistance_ = tempDistance;

                    llLastDataTime_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::system_clock::now().time_since_epoch()).count();

                    strBufferData_ = strBufferData_.substr(head_index + 13);
                }
                else
                {   // 其他消息先不管
                    strBufferData_ = strBufferData_.substr(head_index+1);
                    break;
                }
            }
            else if ((head_index = strBufferData_.find(0xEE)) > -1)
            {
                if((int)strBufferData_.size() < head_index + 9) // 该消息有9个字节
                {
                    break;
                }
                uint16_t tempError = (uint16_t)( ((strBufferData_[head_index + 6] & 0xFF) << 8)
                        | (strBufferData_[head_index + 7] & 0xFF) );
                uiLastError_ = tempError;
                strBufferData_ = strBufferData_.substr(head_index + 9);
//                qDebug() << ErrorValue2String(uiLastError_).c_str();
            }
            else
            {
                if((int)strBufferData_.size() < 100) // 该消息有9个字节
                {
                    strBufferData_.clear();
                }
                break;
            }
        }
    }
}

// 错误值转文字
std::string LaserRangingJrtBb2x::ErrorValue2String(uint16_t error_value)
{
    std::string strError;
    switch (error_value) {
    case 0xFFFF:
    {
        strError = "输入电压过低，输入电压应该大于等于2.2V";
        break;
    }
    case 0xFFFE:
    {
        strError = "内部错误，可忽略";
        break;
    }
    case 0xFFFD:
    {
        strError = "模块温度过低(< -20℃)";
        break;
    }
    case 0xFFFC:
    {
        strError = "模块温度过高(> +40℃)";
        break;
    }
    case 0xFFFB:
    {
        strError = "测量目标超出量程";
        break;
    }
    case 0xFFFA:
    {
        strError = "无效测量结果";
        break;
    }
    case 0xFFF9:
    {
        strError = "背景光太强";
        break;
    }
    case 0xFFF8:
    {
        strError = "反射信号太弱";
        break;
    }
    case 0xFFF7:
    {
        strError = "反射信号太强";
        break;
    }
    case 0xFFF6:
    {
        strError = "硬件错误1";
        break;
    }
    case 0xFFF5:
    {
        strError = "硬件错误2";
        break;
    }
    case 0xFFF4:
    {
        strError = "硬件错误3";
        break;
    }
    case 0xFFF3:
    {
        strError = "硬件错误4";
        break;
    }
    case 0xFFF2:
    {
        strError = "硬件错误5";
        break;
    }
    case 0xFFF1:
    {
        strError = "反射信号不稳定";
        break;
    }
    case 0xFFF0:
    {
        strError = "硬件错误6";
        break;
    }
    case 0xFFEF:
    {
        strError = "硬件错误7";
        break;
    }
    case  0xFF7F:
    {
        strError = "无效结构";
        break;
    }
    default:
        break;
    }
    return strError;
}

}
