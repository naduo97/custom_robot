#include "task_management.h"

namespace TaskManagement{
TaskManagement::TaskManagement(STATE_CALLBACK call_back)
{
    pStateCallback_ = call_back;
    TaskStateData_.TaskState_ = TaskState::Normal;
    pStateCallback_(TaskStateData_);
}

TaskManagement::~TaskManagement()
{
    TaskStateData_.TaskState_ = TaskState::Uninitialized;
}

void TaskManagement::AddTask(const TaskData &task_data)
{
    queTaskDatas_.push(task_data);
    TaskStateData_.TaskState_ = TaskState::Normal;
    pStateCallback_(TaskStateData_);
}

bool TaskManagement::GetANewTask(TaskData &task_data)
{
    if (!queTaskDatas_.empty())
    {
        task_data = queTaskDatas_.front();
        queTaskDatas_.pop();
        TaskStateData_.CurrentTask_ = task_data;
        TaskStateData_.TaskState_ = TaskState::Normal;
        pStateCallback_(TaskStateData_);
        return true;
    }
    TaskStateData_.CurrentTask_.Clear();
    pStateCallback_(TaskStateData_);
    return false;
}

}
