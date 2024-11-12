#include "windowClass.h"

#include <QGridLayout>
#include <QLabel>
#include <QTimer>

Window::Window(){

    Widget *native = new Widget(&helper, this);

    QGridLayout *layout = new QGridLayout;
    layout->addWidget(native, 0, 0);
    setLayout(layout);

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, native, &Widget::animate);
    timer->start(50);
}
