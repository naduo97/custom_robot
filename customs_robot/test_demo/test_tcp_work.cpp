#include "test_tcp_work.h"
#include <QDebug>
namespace TestDemo{
TestTcpWork::TestTcpWork(STATE_CALLBACK call_back)
    : CustomsRobot::WorkBase(CustomsRobot::WhileType::TIME)
    , pTestCallback_(call_back)
{
    // tcp init
    std::string ip = "192.168.1.10";
    TcpClient::SetServerIP(ip);
    TcpClient::SetServerPort(10782);
    TcpClient::Start();
    WorkBase::SetSleep(100);
    WorkBase::ExecutionThread();
}

TestTcpWork::~TestTcpWork()
{
    pTestCallback_ = nullptr;
    qDebug() << "~test_tcp_work";
}

void TestTcpWork::WorkFun()
{
    if (!qData_.empty())
    {
        auto data = qData_.front();
        qData_.pop();
        TcpClient::SendData(data.c_str(), data.size());
    }
    if (pTestCallback_)
    {
        pTestCallback_(testData_);
    }

//    std::string data = "hello";
//    TcpClient::SendData(data.c_str(), data.size());
}

void TestTcpWork::DataProcessing(const std::string &data)
{
    qDebug() << "server data is : " << data.c_str();
    qData_.push(data);
    testData_.strData_ = data;
}

bool TestTcpWork::CheckValidData(std::string &data, int &valid_start, int &valid_size)
{
    int s = -1, e = -1;
    s = data.find("k");
    e = data.find("b");
    if (s >= 0 && e > 0)
    {
        valid_start = s;
        valid_size = e - s + 1;
        return true;
    }
    valid_start = 0;
    valid_size = 0;
    return false;
}
}
