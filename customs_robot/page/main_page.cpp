#include "main_page.h"
#include "ui_main_page.h"
#include <QDebug>
#include <QImage>

MainPage::MainPage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainPage)
{
    ui->setupUi(this);
    ui->liveStream->clear();
    QPalette palette;
    palette.setColor(QPalette::Window, QColor(200, 200, 200));
    ui->liveStream->setAutoFillBackground(true);
    ui->liveStream->setPalette(palette);
    ui->liveStream->setMinimumSize(704, 396);

    ui->liveType->addItem(u8"彩色图");
    ui->liveType->addItem(u8"深度图");
}

MainPage::~MainPage()
{
    delete ui;
}

void MainPage::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
}

void MainPage::UpdateLiveStream(const cv::Mat& mat_data)
{
    QImage neworiginalImage = QImage((const unsigned char*)(mat_data.data), \
                             mat_data.cols, mat_data.rows, \
                             mat_data.step, QImage::Format_RGB888);
    if (!neworiginalImage.isNull())
    {
        ui->liveStream->setPixmap(QPixmap::fromImage(neworiginalImage.scaled(ui->liveStream->width(), ui->liveStream->height(),Qt::KeepAspectRatio, Qt::FastTransformation)));
    }
    else
    {
//       qDebug() << __LINE__ << __FILE__ << "Image is null";
    }
//    QImage newImgshow = newImg.scaled(ui->originalImage->width(), ui->originalImage->height(),Qt::KeepAspectRatio, Qt::FastTransformation);
     
}

void MainPage::on_liveType_currentIndexChanged(int index)
{
    qDebug() << index;
    emit LiveTypeChange(index);
}
