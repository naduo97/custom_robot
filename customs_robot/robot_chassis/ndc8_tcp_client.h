#ifndef NDC8TCPCLIENT_H
#define NDC8TCPCLIENT_H
#include "tcp_client.h"
#include "chassis_state_data.h"
#include <functional>

#include <libxml/parser.h>
#include <libxml/tree.h>


namespace RobotChassis{

typedef std::function<void(const std::string &attr_name, const std::string &attr_value, const std::string &value)> NDC8_DATA_CALLBACK;

class NDC8TcpClient : public CustomsRobot::TcpClient
{
public:
    NDC8TcpClient(NDC8_DATA_CALLBACK callback);
    ~NDC8TcpClient();
    void DataProcessing(const std::string &data);
    bool CheckValidData(std::string &data, int &valid_start, int &valid_size);
    void ParseXmlData(xmlNodePtr node);

public:
    NDC8_DATA_CALLBACK pDataCallback_;
};
}
#endif // NDC8TCPCLIENT_H
