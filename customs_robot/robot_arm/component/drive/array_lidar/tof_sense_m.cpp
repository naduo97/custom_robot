#include "tof_sense_m.h"

namespace RobotArm{
namespace component{
TOFSenseM::TOFSenseM(std::string device_name, int baud_rate) :
    Port_(device_name, baud_rate)
{
    bStop_ = false;
    thUpdata_ = std::thread(&TOFSenseM::ThreadFun, this);
}

TOFSenseM::~TOFSenseM()
{
    bStop_ = true;
    Port_.StopCommunication();
    if (thUpdata_.joinable())
    {
        thUpdata_.join();
    }
}

uint8_t TOFSenseM::VerifyCheckSum(const void *data, size_t data_length)
{
  const uint8_t *byte = (uint8_t *)data;
  uint8_t sum = 0;
  for (size_t i = 0; i < data_length - 1; ++i)
  {
    sum += byte[i];
  }
  return sum == byte[data_length - 1];
}

void TOFSenseM::ThreadFun()
{
    while(!bStop_)
    {
        if (!Port_.IsCommunication())
        {
            Port_.EstablishCommunication();
        }
        std::string strData;
        Port_.ReadData(strData);
        strBufferData_ += strData;
        int head_index = -1;
        if ((head_index = strBufferData_.find(0x57)) > -1)
        {
            if ((head_index + 9) > (int)strBufferData_.size())
            {
                continue;
            }
            const TOFSenseMData *pDataFrame = (const TOFSenseMData *)strBufferData_.c_str()[head_index];
            if (pDataFrame->function_mark != 0x01)
            {
                strBufferData_ = strBufferData_.substr(head_index + 1);
                continue;
            }
            if (pDataFrame->pixel_count != 64)
            {
                strBufferData_ = strBufferData_.substr(head_index + 1);
                continue;
            }
            uint64_t uiDataLeng = sizeof(*pDataFrame) - sizeof(pDataFrame->pixels) + sizeof(pDataFrame->pixels[0]) * pDataFrame->pixel_count;
            if (uiDataLeng <= strBufferData_.size())
            {
                continue;
            }
            if (!VerifyCheckSum((void *)pDataFrame, uiDataLeng))
            {
                strBufferData_ = strBufferData_.substr(head_index + 1);
                continue;
            }


        }
    }
}

}}
