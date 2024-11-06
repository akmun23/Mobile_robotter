#include "widgetClass.h"

#include <QPainter>
#include <QTimer>

Widget::Widget(Helper *helper, QWidget *parent) : QWidget(parent), helper(helper){
    elapsed = 0;
    setFixedSize(getScrnSize());
}

void Widget::animate(){
    elapsed = (elapsed + qobject_cast<QTimer*>(sender())->interval()) % 1000;
    update();
}

void Widget::paintEvent(QPaintEvent *event){
    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);
    helper->lidarRead(rand()%8, 100);
    helper->paint(&painter, event, elapsed);
    helper->paintRobot(&painter, event, elapsed);
    painter.end();
}
