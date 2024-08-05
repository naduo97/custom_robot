#include "image_check_box.h"
#include <QEvent>
#include <QPainter>
#include <QDebug>

ImageCheckBox::ImageCheckBox(QWidget *p)
    :QCheckBox(p)
{}

void ImageCheckBox::mousePressEvent(QMouseEvent *e)  //处理按下事件
{
    if (this->isChecked())
    {
        this->setIcon(_icons[0]);
    }
    else
    {
        this->setIcon(_icons[3]);
    }

    setIconSize(size());
    QCheckBox::mousePressEvent(e);//交给父类 继续执行按钮的其他 事件。不调用可能导致父类功能缺失
}

void ImageCheckBox::mouseReleaseEvent(QMouseEvent *e) //处理释放事件
{
    if (this->isChecked())
    {
        this->setIcon(_icons[1]);
    }
    else
    {
        this->setIcon(_icons[2]);
    }
    setIconSize(size());
    QCheckBox::mouseReleaseEvent(e);//交给父类 继续执行按钮的其他 事件。不调用可能导致父类功能缺失
}

bool ImageCheckBox::event(QEvent *e)                 //处理鼠标进入按钮事件
{
    if(e->type()==QEvent::HoverEnter)//进来 入
    {
        if (this->isChecked())
        {
            this->setIcon(_icons[2]);
        }
        else
        {
            this->setIcon(_icons[1]);
        }
//        setIcon(_icons[1]);
        setIconSize(size());
    }
    else if(e->type()==QEvent::HoverLeave)
    {//离开 出
        if (this->isChecked())
        {
            this->setIcon(_icons[3]);
        }
        else
        {
            this->setIcon(_icons[0]);
        }
        auto temp_size = size();
        setIconSize(temp_size);
    }
    return QCheckBox::event(e); //交给父类 继续执行按钮的其他 事件。不调用可能导致父类功能缺失
}

void ImageCheckBox::paintEvent(QPaintEvent *event)
{
   Q_UNUSED(event);
   QPainter painter(this);
   QPixmap pixmap = icon().pixmap(size());  // 获取图像的pixmap
   // 绘制文字
   painter.drawPixmap(rect(), pixmap);  // 绘制图像的pixmap
//   painter.setRenderHints(QPainter::Antialiasing, true);
}

bool ImageCheckBox::setImage(QList<QIcon>& icons)
{
    if(icons.size()!=4)
    {
        return false;
    }
    else{
        _icons=icons;
        setIcon(_icons[0]);
        setIconSize(size());
    }

    return true;
}
