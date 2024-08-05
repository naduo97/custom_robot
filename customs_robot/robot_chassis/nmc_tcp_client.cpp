#include "nmc_tcp_client.h"
#include <QDebug>


namespace RobotChassis{
NMCTcpClient::NMCTcpClient(NMC_DATA_CALLBACK callback)
{
    pDataCallback_ = callback;
}

NMCTcpClient::~NMCTcpClient()
{
    pDataCallback_ = nullptr;
}

// 消息处理
void NMCTcpClient::DataProcessing(const std::string &data)
{
    if (nullptr != pDataCallback_)
    {
       pDataCallback_(data);
    }
}

// 消息验证
bool NMCTcpClient::CheckValidData(std::string &data, int &valid_start, int &valid_size)
{
    const char* pData = data.c_str();
    for (int index = 0; index < (int)data.size(); index++)
    {
        switch (pData[index])
        {
            case 0x73:  //  雷达数据
            {
                uint16_t data_leng = 0;
                if (index + 3 < (int)data.size())   // 获取长度
                {
                    data_leng = pData[index+3];
                    data_leng <<= 8;
                    data_leng |= 0x00FF;
                    data_leng &= pData[index+2];
                    qDebug() << data_leng;
                }
                if ( data_leng == 0 || (index + 4 + (int)data_leng < (int)data.size()) )
                {
                    break;
                }

                valid_start = index;
                valid_size = (int)data_leng;
                return true;
            }
            default :
            {
                break;
            }
        }
    }

    valid_start = 0;
    valid_size = 0;
    return false;
}
}
