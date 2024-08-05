# 海关项目-上位机
## 模块模板说明 
--------------------------------------  
* CustomsRobot::TcpClient  
>类功能：建立TCP客户端与指定服务器进行连接，并创建线程监听来自服务器的消息。同时可调用相关函数向服务器发送消息。TCP无法与服务器连接时，会间隔一定时间再尝试与服务连接。直到类析构函数被调用时，线程才会结束。  
>头文件: #include "tcp_client.h"    
>主要函数说明：  
>>    - void Start();  
>>>    该函数会创建一个线程不断尝试与服务器建立连接，直到连接成功或类析构被调用。但该类不会堵塞运行。  
>>    - void SetServerIP(std::string &server_ip);    
>>>    设置服务器IP  
>>    - void SetServerPort(int port);    
>>>    设置服务器端口  
>>    - void SetBuffSize(int size);  
>>>    设置每次从通讯管道中读取数据的大小，该值默认1024。  
>>    - bool IsConnectServer();  
>>>    获取当前与服务器是否正常，正常：true  不正常：false  
>>    - bool SendData(const char *data, int size);  
>>>    向服务器发送消息  
>>    - ___virtual bool CheckValidData(std::string &data, int , int &valid_size);___  
>>>    **该函数相当重要，所有继承该类的子类，必须要重写该函数。**  
>>>    该函数需要子类根据自身的消息协议，查验并给出正确的有效数据范围。如果"data"有问题，"valid_start"需要赋值为 0 或 不修改该值，而"valid_size"同样赋值为 0 或 不修改该值。
>>    - ___virtual void DataProcessing(const std::string &data);___  
>>>    **该函数相当重要，所有继承该类的子类，必须要重写该函数。**  
>>>    接收到服务器的消息并通过了CheckValidData()函数的查验，该函数就会被调用,传参内容是CheckValidData()函数所决定的有效数据。  
    
---------------------------------  
* CustomsRobot::WorkBase  
>类功能：根据用户的要求，建立时间等待或信号触发的任务线程。直到类析构函数被调用时，线程才会结束。  
>头文件: #include "work_base.h"  
>该类创建时就需要指定工作的等待类型，WhileType::TIME 或 WhileType::SEMAPHORE  
>>   - WhileType::TIME  
>>>   表示按照时间睡眠来控制线程的频率。  
>>   - WhileType::SEMAPHORE  
>>>   表示按信号触发来控制线程的频率。触发函数 SemaphoreActivation()  

>主要函数说明：  
>>   - bool ExecutionThread();  
>>>   该函数会启动一个工作线程。  
>>   - void SetSleep(int time);  
>>>   设置每轮线程的等待时间
>>   - void SemaphoreActivation();  
>>>   激活一次等待线程
>>   - ___virtual void WorkFun();___  
>>>   **该函数相当重要，所有继承该类的子类，必须要重写该函数。**  
>>>   子类需要重写该函数，并在该函数里完成自己所需要完成的任务。

