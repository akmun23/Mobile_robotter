#ifndef HELPERCLASS_H
#define HELPERCLASS_H

#include <QBrush>
#include <QFont>
#include <QPen>
#include <QWidget>

#include "wallClass.h"
#include "functions.h"

class Helper{

public:
    Helper();

    void paint(QPainter *painter, QPaintEvent *event, int elapsed);
    void paintRobot(QPainter *painter, QPaintEvent *event, int elapsed);
    void lidarRead(float angle, float len);
    int robotPos();

private:

    struct{
        float x = (getScrnWidth()/2);
        float y = (getScrnHeight()/2);
        float center_x = x + 25;
        float center_y = y + 25;
    }robot;

    QBrush background;
    QBrush wall;
    QBrush robotBrush;
    QBrush empty;

    QList<Wall> wallFrags; //Skal være af en klasse som kan holde styr på koords, farve/type og orientering
};

#endif // HELPERCLASS_H
