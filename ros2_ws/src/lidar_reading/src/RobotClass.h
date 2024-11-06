#ifndef ROBOTCLASS_H
#define ROBOTCLASS_H

#include <QApplication>
#include <QtWidgets>
#include <QWindow>
#include <qbrush.h>
#include <vector>
#include "math.h"

class Robot{
private:
    int x, y;
    std::vector<int> orient;
    QList<QRect> roboRects;

public:

    Robot(){
        x = 100;
        y = 100;

        orient.push_back(0);
        orient.push_back(1);

        roboRects.push_back(QRect(x, y + 25, 25, 50));
        roboRects.push_back(QRect(x + 25, y, 50, 25));
    }

    QList<QRect> getRects(){
        return roboRects;
    }

    void rotate(int degrees){
        orient[1] = cos(degrees);
        orient[2] = sin(degrees); //skal lige fikses

        x = x * orient[1];
        y = y * orient[2];
    }

};

#endif // ROBOTCLASS_H
