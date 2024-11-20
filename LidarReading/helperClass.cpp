#include "helperClass.h"
#include "wallClass.h"
#include "math.h"
#include <iostream>
#include <QPainter>
#include <QPaintEvent>
#include <QWidget>

using namespace std;

Helper::Helper() {
    QSize sqSize(5, 5);
    background = QBrush(Qt::black);
    wall = QBrush(QColor(0, 255, 255));
    empty = QBrush(QColor(255, 255, 255));
    robotBrush = QBrush(Qt::red);

    for (int i = 0; i < (getScrnHeight() / sqSize.height()) - 20; i++) {
        for (int j = 0; j < getScrnWidth() / sqSize.width(); j++) {
            wallFrags.append(Wall(QPoint(j * sqSize.width(), i * sqSize.height()), sqSize, 0));
        }
    }
}

void Helper::paint(QPainter *painter, QPaintEvent *event, int elapsed) {
    for (int i = 0; i < wallFrags.size(); i++) {
        if (wallFrags[i].getType() == Wall::typeWall) {
            painter->setBrush(wall);
            painter->drawRect(wallFrags[i]);
        } else if (wallFrags[i].getType() == Wall::typeEmpty) {
            painter->setBrush(empty);
            painter->drawRect(wallFrags[i]);
        } else if (wallFrags[i].getType() == Wall::typeBackground) {
            painter->setBrush(background);
            painter->drawRect(wallFrags[i]);
        }
    }
}

void Helper::paintRobot(QPainter *painter, QPaintEvent *event, int elapsed) {
    painter->setBrush(robotBrush);
    painter->drawRect(QRect(QPoint(robot.center_x, robot.center_y), QSize(50, 50)));
}

int Helper::robotPos() {
    for (int i = 0; i < wallFrags.size(); i++) {
        if (wallFrags[i].contains(robot.center_x, robot.center_y)) {
            return i;
        }
    }
    return 0;
}

void Helper::lidarRead(float angle, float distance) {
    float x_com = angle*100;
    float y_com = distance*100;

    // Calculate the end position of the LIDAR reading
    float end_x = robot.center_x + x_com;
    float end_y = robot.center_y + y_com;

    // Iterate through the wall fragments and update their types
    for (int i = 0; i < wallFrags.size(); i++) {
        if (wallFrags[i].contains(end_x, end_y)) {
            wallFrags[i].setType(Wall::typeWall);

            // Mark the fragments between the robot and the detected wall as empty
            float step_x = x_com / distance;
            float step_y = y_com / distance;
            for (int j = 0; j < distance; j++) {
                float current_x = robot.center_x + step_x * j;
                float current_y = robot.center_y + step_y * j;
                for (int k = 0; k < wallFrags.size(); k++) {
                    if (wallFrags[k].contains(current_x, current_y)) {
                        wallFrags[k].setType(Wall::typeEmpty);
                    }
                }
            }
        }
    }
}
