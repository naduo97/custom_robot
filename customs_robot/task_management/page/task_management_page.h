#ifndef TASK_MANAGEMENT_PAGE_H
#define TASK_MANAGEMENT_PAGE_H

#include <QWidget>
#include <QSettings>
#include <unordered_map>
#include <QTableWidgetItem>
#include "task_management/task_management.h"

namespace Ui {
class TaskManagementPage;
}

class TaskManagementPage : public QWidget
{
    Q_OBJECT

public:
    explicit TaskManagementPage(QWidget *parent = nullptr);
    ~TaskManagementPage();
    void LoadConfig(QSettings &configIni);
    void Update(const TaskManagement::TaskStateData &data);
private:
    void Init(QSettings &config);

private:
    Ui::TaskManagementPage *ui;
    std::unordered_map<std::string, TaskManagement::TaskData> umapTaskData_;

signals:
    void SendTaskData(const TaskManagement::TaskControl &data);

private slots:
    void on_taskPushButton_clicked();
    void on_complete_mountingButton_clicked();
    void on_complete_unmountingButton_clicked();
    void on_complete_loadingButton_clicked();
public:
    std::unordered_map<int, QTableWidgetItem*> umapListData_;
};

#endif // TASK_MANAGEMENT_PAGE_H
