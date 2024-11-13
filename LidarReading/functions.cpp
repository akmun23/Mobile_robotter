#include "functions.h"
#include <qapplication.h>

int getScrnWidth(){
    QSize size = qApp->screens()[0]->size();
    return size.width();
}

int getScrnHeight(){
    QSize size = qApp->screens()[0]->size();
    return size.height();
}

QSize getScrnSize(){
    QSize size = qApp->screens()[0]->size();
    size.setWidth(size.width()-90); //Remove width of taskbar in ubuntu version 22.04
    return size;
}
