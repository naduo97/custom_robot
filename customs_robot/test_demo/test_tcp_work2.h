#ifndef TESTTCPCLIENT2_H
#define TESTTCPCLIENT2_H

#include "work_base.h"
#include "tcp_client.h"
#include <functional>
namespace TestDemo{

class Test2StateData
{
public:
    Test2StateData(){}
    ~Test2StateData(){}
    std::string strData_;
};

typedef std::function<void(const Test2StateData&)> TEST2_STATE_CALLBACK;

class TestTcpWork2
    : public CustomsRobot::WorkBase
    , public CustomsRobot::TcpClient
{
public:
    TestTcpWork2(TEST2_STATE_CALLBACK callback);
    ~TestTcpWork2();

    void WorkFun();
    void DataProcessing(const std::string &data);
    bool CheckValidData(std::string &data, int &valid_start, int &valid_size);

public:
    Test2StateData testData_;
    TEST2_STATE_CALLBACK pTest2StateCallback_;
//    std::queue<std::string> qData_;
};

}

#endif // TESTTCPCLIENT2_H
