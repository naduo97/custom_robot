#ifndef TOFSENSEM_H
#define TOFSENSEM_H
#include "stdint.h"
#include <thread>
#include "serial_communication.h"
#include <cstring>

namespace RobotArm{
namespace component{

#pragma pack(1)

typedef struct
{
  uint8_t byteArray[3];
} nint24_t;

typedef struct
{
  uint8_t byteArray[3];
} nuint24_t;

typedef struct
{
  nint24_t dis;
  uint8_t dis_status;
  uint16_t signal_strength;
} PixelData;

struct TOFSenseMData
{
    uint8_t frame_header;
    uint8_t function_mark;
    uint8_t reserved;
    uint8_t id;
    uint32_t system_time;
    uint8_t pixel_count;
    PixelData pixels[64];
    uint8_t reserved1[6];
    uint8_t sum;
    void operator=(const TOFSenseMData& other)
    {
        this->frame_header = other.frame_header;
        this->function_mark = other.function_mark;
        this->reserved = other.reserved;
        this->id = other.id;
        this->system_time = other.system_time;
        this->pixel_count = other.pixel_count;
        std::memcpy((void*)other.pixels, (void*)this->pixels, 64 * sizeof(PixelData));
        std::memcpy((void*)other.reserved1, (void*)this->reserved1, 6 * sizeof(uint8_t));
        this->sum = other.sum;
    }
};
#pragma pack()

class TOFSenseM
{
public:
    TOFSenseM(std::string device_name, int baud_rate);
    ~TOFSenseM();
private:
    uint8_t VerifyCheckSum(const void *data, size_t data_length);
    void ThreadFun();
private:
    bool bStop_;
    std::string strBufferData_;
    CustomsRobot::SerialCommunication  Port_;
    std::thread thUpdata_;
};

}}
#endif // TOFSENSEM_H
