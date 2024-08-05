#ifndef MAIN_PAGE_H
#define MAIN_PAGE_H

#include <QWidget>
#include <opencv2/opencv.hpp>

namespace Ui {
class MainPage;
}

class MainPage : public QWidget
{
    Q_OBJECT

public:
    explicit MainPage(QWidget *parent = nullptr);
    ~MainPage();

    void UpdateLiveStream(const cv::Mat& mat_data);
protected:
    virtual void resizeEvent(QResizeEvent *event) override;

private slots:
    void on_liveType_currentIndexChanged(int index);

signals:
    void LiveTypeChange(const int &type);

private:
    Ui::MainPage *ui;
};

#endif // MAIN_PAGE_H
