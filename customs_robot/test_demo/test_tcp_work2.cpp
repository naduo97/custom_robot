#include "test_tcp_work2.h"
#include "QDebug"
namespace TestDemo{
TestTcpWork2::TestTcpWork2(TEST2_STATE_CALLBACK callback)
    : CustomsRobot::WorkBase(CustomsRobot::WhileType::SEMAPHORE)
    , pTest2StateCallback_(callback)
{
    std::string ip = "192.168.1.10";
    TcpClient::SetServerIP(ip);
    TcpClient::SetServerPort(10782);
    TcpClient::Start();
    WorkBase::ExecutionThread();
}


TestTcpWork2::~TestTcpWork2()
{
    qDebug() << "~TestTcpClient2";
    pTest2StateCallback_ = nullptr;
}


void TestTcpWork2::WorkFun()
{
    qDebug() << "TestTcpClient2::WorkFun";
    if (pTest2StateCallback_)
    {
        pTest2StateCallback_(testData_);
    }
}

void TestTcpWork2::DataProcessing(const std::string &data)
{
    testData_.strData_ = data;
    WorkBase::SemaphoreActivation();
}


bool TestTcpWork2::CheckValidData(std::string &data, int &valid_start, int &valid_size)
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
