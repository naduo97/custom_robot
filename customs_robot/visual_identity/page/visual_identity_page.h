#ifndef VISUAL_IDENTITY_PAGE_H
#define VISUAL_IDENTITY_PAGE_H

#include <QWidget>
#include <QSettings>
#include "visual_identity/visual_identity.h"

namespace Ui {
class VisualIdentityPage;
}

class VisualIdentityPage : public QWidget
{
    Q_OBJECT

public:
    explicit VisualIdentityPage(QWidget *parent = nullptr);
    ~VisualIdentityPage();

    // 加载配置
    void LoadConfig(QSettings &configIni);
    // 更新数据
    void Update(const VisualIdentity::VisualIdentityStateData &data);
private slots:
    void on_artificialData_clicked();

    void on_btn_BoxDetection_clicked();

    void on_btnConfirmVisualResults_clicked();

    void on_btnRetakeRecognition_clicked();

signals:
    void ModifyingArtificialData(bool data);
    void SendVisualIdentityControl(const VisualIdentity::VisualIdentityControl &data);

private:
    Ui::VisualIdentityPage *ui;
};

#endif // VISUAL_IDENTITY_PAGE_H
