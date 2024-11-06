#ifndef WINDOWCLASS_H
#define WINDOWCLASS_H

#include "widgetClass.h"

#include <QWidget>

class Window : public QWidget
{
    Q_OBJECT

public:
    Window();

private:
    Helper helper;
};

#endif // WINDOWCLASS_H
