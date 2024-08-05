#ifndef TASKMANAGEMENT_H
#define TASKMANAGEMENT_H
#include <string>
#include <queue>
#include <functional>
#include "common/common.h"

namespace TaskManagement{

class TaskData
{
public:
    TaskData()
    {
        this->strText_ = "";
        this->iCarriageId_ = 0;
        this->iWorkType_ = 0;
        this->Pose_.SetZero();
    }

    ~TaskData(){}

    void Clear()
    {
        this->strText_ = "";
        this->iCarriageId_ = 0;
        this->iWorkType_ = 0;
        this->Pose_.SetZero();
    }

    TaskData& operator=(const TaskData& data)
    {
        this->strText_ = data.strText_;
        this->iCarriageId_ = data.iCarriageId_;
        this->iWorkType_ = data.iWorkType_;
        this->Pose_ = data.Pose_;
        return *this;
    }

public:
    std::string strText_;
    int iCarriageId_;
    int iWorkType_;
    Common::Pose Pose_;
};

enum TaskState
{
    Uninitialized = 0,      //  未初始化
    Normal = 1,             //  正常
    ERROR = 2               //  错误
};


class TaskControl
{
public:
    enum ControlType
    {
        Init = 0,           //
        PublishTask = 1,    //
        CompleteMounting = 2, //
        CompleteUnMounting = 3, //
        CompleteLoading = 4
    };
public:
    TaskControl(){}
    ~TaskControl(){}

public:
    ControlType ControlType_;
    TaskData TaskData_;
};


class TaskStateData
{
public:
    TaskStateData() : TaskState_(TaskState::Uninitialized){}
    ~TaskStateData(){}
public:
    TaskState TaskState_;
    TaskData CurrentTask_;
};

typedef std::function<void(const TaskStateData&)> STATE_CALLBACK;

class TaskManagement
{
public:
    TaskManagement(STATE_CALLBACK call_back);
    ~TaskManagement();

public:
    bool GetANewTask(TaskData &task_data);
    void AddTask(const TaskData &task_data);
private:
    STATE_CALLBACK pStateCallback_;
    TaskStateData TaskStateData_;
    std::queue<TaskData> queTaskDatas_;
};
}
#endif // TASKMANAGEMENT_H
