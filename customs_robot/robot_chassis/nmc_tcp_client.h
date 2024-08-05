#ifndef NMCTCPCLIENT_H
#define NMCTCPCLIENT_H
#include "tcp_client.h"
#include "chassis_state_data.h"
#include <functional>

namespace RobotChassis{

typedef std::function<void(const std::string&)> NMC_DATA_CALLBACK;

class NMCTcpClient : public CustomsRobot::TcpClient
{
public:
    NMCTcpClient(NMC_DATA_CALLBACK callback);
    ~NMCTcpClient();
    void DataProcessing(const std::string &data);
    bool CheckValidData(std::string &data, int &valid_start, int &valid_size);

public:
    NMC_DATA_CALLBACK pDataCallback_;
};

}
#endif // NMCTCPCLIENT_H
