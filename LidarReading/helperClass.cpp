#include "helperClass.h"
#include "wallClass.h"
#include "math.h"

#include <iostream>
#include <QPainter>
#include <QPaintEvent>
#include <QWidget>

using namespace std;

Helper::Helper(){
    QSize sqSize(5,5);

    background = QBrush(Qt::black);
    wall = QBrush(QColor(120,120,120));
    empty = QBrush(QColor(255,255,255));
    robotBrush = QBrush(Qt::red);

    //Initialization of grid - Would be nice with QSize instead of integer (5)
    for(int i = 0; i < (getScrnHeight()/sqSize.height()) - 20; i++){
        for(int j = 0; j < getScrnWidth()/sqSize.width(); j++){
            wallFrags.append(Wall(QPoint(j*sqSize.width(),i*sqSize.height()), sqSize, 0));
        }
    }
}

void Helper::paint(QPainter *painter, QPaintEvent *event, int elapsed){

    for(int i = 0; i < wallFrags.size(); i++){

        if(wallFrags[i].getType() == Wall::typeWall){
            painter->setBrush(wall);
            painter->drawRect(wallFrags[i]);
        }
        else if(wallFrags[i].getType() == Wall::typeEmpty){
            painter->setBrush(empty);
            painter->drawRect(wallFrags[i]);
        }
        else if(wallFrags[i].getType() == Wall::typeBackground){
            painter->setBrush(background);
            painter->drawRect(wallFrags[i]);
        }
    }
}

void Helper::paintRobot(QPainter *painter, QPaintEvent *event, int elapsed){
    painter->setBrush(robotBrush);
    painter->drawRect(QRect(QPoint(robot.center_x, robot.center_y), QSize(10,10)));
}

int Helper::robotPos(){
    for(int i = 0; i < wallFrags.size(); i++){
        if(wallFrags[i].contains(robot.center_x, robot.center_y)){
            return i;
        }
    }

    return 0;
}

void Helper::lidarRead(float angle, float len){
    float x_com, y_com;

    angle = angle*(180/M_PI);

    x_com = len*cos(angle);
    y_com = len*sin(angle);

    if(angle >= M_PI || angle <= M_PI){
        for(int i = 0; i < wallFrags.size(); i++){ //If angle < pi
            if(wallFrags[i].contains(robot.center_x + x_com, robot.center_y + y_com)){
                robot.center_x = robot.center_x + 1;
                //robot.center_y = robot.center_y + 1;

                wallFrags[i].setType(Wall::typeWall);

                if(y_com >= 0){
                    for(int ii = 0; ii < i; ii++){//Check if there are any walls inbetween the robot and detected wall
                        for(int iii = 0; iii < len; iii++){
                            if(wallFrags[ii].contains((robot.center_x + (x_com/len)*iii), (robot.center_y + (y_com/len)*iii))){
                                wallFrags[ii].setType(Wall::typeEmpty);
                            }
                        }
                    }
                }
                else{
                    for(int ii = robotPos(); i < ii; ii--){//Check if there are any walls inbetween the robot and detected wall
                        for(int iii = 0; iii < len; iii++){
                            if(wallFrags[ii].contains((robot.center_x + (x_com/len)*iii), (robot.center_y + (y_com/len)*iii))){
                                wallFrags[ii].setType(Wall::typeEmpty);
                            }
                        }
                    }
                }
            }
        }
    }
}
