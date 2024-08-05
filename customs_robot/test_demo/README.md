# 测试Demo说明
## TestDemo::TestTcpWork
>对象功能：
>>* 创建一个线程通过TCP与指定服务器进行通讯。  
>>* 收到服务器消息后，先通过消息校验，再将消息保存到队列中。
>>* 另一个线程每隔100ms向队列中获取消息并刷新到交互界面。
>>* 交互界面有一个按钮，当点击该按钮时，向服务器发送一个“hello”字符串（发送案例暂不考虑消息格式）。  
>-----------------------------------------------------------------
>实现说明：  
>>* 创建独有的状态对象。每个设备对象都有自己独有的状态对象
>>> TestDemo::TestStateData
>>* 声明状态对象的回调函数指针
>>> ```cpp
>>>typedef std::function<void(const TestStateData&)> STATE_CALLBACK;
>>>// 用户需要根据每个状态类创建不同的回调函数
>>>```
>>-----------------------------------------------------------------
>>* 继承"CustomsRobot::TcpClient"和"CustomsRobot::WorkBase"两个类  
>>* 构造函数初始化  
>>>```cpp
>>>TestTcpWork::TestTcpWork(STATE_CALLBACK call_back)
>>>    : CustomsRobot::WorkBase(CustomsRobot::WhileType::TIME)   //时间等待
>>>    , pTestCallback_(call_back)
>>>{
>>>    // 设置网络信息
>>>    std::string ip = "192.168.1.10";
>>>    TcpClient::SetServerIP(ip);
>>>    TcpClient::SetServerPort(10782);
>>>    TcpClient::Start(); // 开启通讯线程
>>>    // 设置工作线程
>>>    WorkBase::SetSleep(100);
>>>    WorkBase::ExecutionThread();// 开启工作线程
>>>}
>>>```
>>* 重写消息校验函数。  
>>>查找"k"开头，"b"结尾的有效消息，并返回有效消息的范围  
>>>```cpp
>>>bool TestTcpWork::CheckValidData(std::string &data, int &valid_start, int &valid_size)
>>>{
>>>    int s = -1, e = -1;
>>>    s = data.find("k");
>>>    e = data.find("b");
>>>    if (s >= 0 && e > 0)
>>>    {
>>>        valid_start = s;
>>>        valid_size = e - s + 1;
>>>        return true;
>>>    }
>>>    valid_start = 0;
>>>    valid_size = 0;
>>>    return false;
>>>}
>>>```  
>>* 重写消息处理函数函数。将收到的消息更新到消息队列中
>>>```cpp  
>>>void TestTcpWork::DataProcessing(const std::string &data)
>>>{
>>>    qData_.push(data);
>>>    testData_.strData_ = data;
>>>}
>>>```
>>* 重写工作函数。从队列中获取一个状态消息，并通过回调函数，将状态更新到交互界面中
>>> ```cpp  
>>>void TestTcpWork::WorkFun()
>>>{
>>>    if (!qData_.empty())
>>>    {
>>>        auto data = qData_.front();
>>>        qData_.pop();
>>>        TcpClient::SendData(data.c_str(), data.size());
>>>    }
>>>    if (pTestCallback_)
>>>    {
>>>        pTestCallback_(testData_);
>>>    }
>>>
>>>}
>>>```  
------  
## TestDemo::TestTcpWork2  
>对象功能：
>>* 创建一个线程通过TCP与指定服务器进行通讯。  
>>* 收到服务器消息后，先通过消息校验，再将消息内容更新到自身的状态类中。
>>* 另一个线程收到信号触发，将状态内容刷新到交互界面。
>---------
>>实现说明：  
>>* 创建独有的状态对象。每个设备对象都有自己独有的状态对象
>>> TestDemo::Test2StateData
>>* 声明状态对象的回调函数指针
>>> ```cpp
>>>typedef std::function<void(const Test2StateData&)> TEST2_STATE_CALLBACK;
>>>// 用户需要根据每个状态类创建不同的回调函数
>>>```
>>-----------------------------------------------------------------
>>* 继承"CustomsRobot::TcpClient"和"CustomsRobot::WorkBase"两个类  
>>* 构造函数初始化  
>>>```cpp
>>>TestTcpWork2::TestTcpWork2(TEST2_STATE_CALLBACK callback)
>>>    : CustomsRobot::WorkBase(CustomsRobot::WhileType::SEMAPHORE) //信号等待
>>>    , pTest2StateCallback_(callback)
>>>{
>>>    // 设置网络信息
>>>    std::string ip = "192.168.1.10";
>>>    TcpClient::SetServerIP(ip);
>>>    TcpClient::SetServerPort(10782);
>>>    TcpClient::Start();
>>>    // 设置工作线程
>>>    WorkBase::ExecutionThread(); // 开启工作线程
>>>}
>>>```
>>* 重写消息校验函数。  
>>>查找"k"开头，"b"结尾的有效消息，并返回有效消息的范围  
>>>```cpp
>>>bool TestTcpWork2::CheckValidData(std::string &data, int &valid_start, int &valid_size)
>>>{
>>>    int s = -1, e = -1;
>>>    s = data.find("k");
>>>    e = data.find("b");
>>>    if (s >= 0 && e > 0)
>>>    {
>>>        valid_start = s;
>>>        valid_size = e - s + 1;
>>>        return true;
>>>    }
>>>    valid_start = 0;
>>>    valid_size = 0;
>>>    return false;
>>>}
>>>```  
>>* 重写消息处理函数函数。将收到的消息更新到状态类中，并激活线程
>>>```cpp  
>>>void TestTcpWork2::DataProcessing(const std::string &data)
>>>{
>>>    testData_.strData_ = data;
>>>    WorkBase::SemaphoreActivation(); // 激活线程
>>>}
>>>```
>>* 重写工作函数。通过回调函数，将状态更新到交互界面中
>>> ```cpp  
>>>void TestTcpWork2::WorkFun()
>>>{
>>>    if (pTest2StateCallback_)
>>>    {
>>>        pTest2StateCallback_(testData_);
>>>    }
>>>}
>>>```  