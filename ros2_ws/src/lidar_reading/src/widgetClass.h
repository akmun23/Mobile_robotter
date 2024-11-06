#ifndef WIDGETCLASS_H
#define WIDGETCLASS_H

#include <QWidget>

#include "helperClass.h"

class Helper;

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(Helper *helper, QWidget *parent);

public slots:
    void animate();

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    Helper *helper;
    int elapsed;
};

#endif // WIDGETCLASS_H
