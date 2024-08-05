#ifndef IMAGECHECKBOX_H
#define IMAGECHECKBOX_H
#include <QCheckBox>

class ImageCheckBox : public QCheckBox
{
    Q_OBJECT
public:
    ImageCheckBox(QWidget *p=nullptr);
    void mousePressEvent(QMouseEvent *e);  //处理按下事件
    void mouseReleaseEvent(QMouseEvent *e);//处理释放事件
    bool event(QEvent *e);                 //处理鼠标进入按钮事件
    void paintEvent(QPaintEvent *event);
public:
    bool setImage(QList<QIcon>& icons);//接受图片方法

private:
    QList<QIcon> _icons;//储存图片路径集合
};

#endif // IMAGECHECKBOX_H
