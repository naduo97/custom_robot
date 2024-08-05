#ifndef TEST_TCP_WORK_H
#define TEST_TCP_WORK_H
#include "work_base.h"
#include "tcp_client.h"
#include <queue>
#include <functional>
namespace TestDemo{

class TestStateData
{
public:
    TestStateData(){}
    ~TestStateData(){}
    std::string strData_;
};

typedef std::function<void(const TestStateData&)> STATE_CALLBACK;

class TestTcpWork
    : public CustomsRobot::WorkBase
    , public CustomsRobot::TcpClient
{
public:

    TestTcpWork(STATE_CALLBACK call_back);
    ~TestTcpWork();
    void WorkFun();
    void DataProcessing(const std::string &data);
    bool CheckValidData(std::string &data, int &valid_start, int &valid_size);

    TestStateData testData_;
    STATE_CALLBACK pTestCallback_;
    std::queue<std::string> qData_;
};

}
#endif // TEST_TCP_WORK_H
