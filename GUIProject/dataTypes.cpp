#include "dataTypes.h"
#include <iostream>

Point::Point(int x_, int y_){
    x = x_;
    y = y_;
}

Point::Point(float x_, float y_){
    x = x_;
    y = y_;
}

Size::Size(int width_, int height_){
    width = width_;
    height = height_;
}

Robot::Robot(int x_, int y_, Size size_){
    x = x_;
    y = y_;
    size = size_;
}

Robot::Robot(Point p, Size size_){
    x = p.x;
    y = p.y;

    size = size_;
    xAxis.width = size.width;
    xAxis.height = size.height * 1.5;

    pointsXAxis[0].x = x - size.width/2;
    pointsXAxis[0].y = y - size.height/2;
    pointsXAxis[1].x = x - size.width/2;
    pointsXAxis[1].y = y + size.height/2;
    pointsXAxis[2].x = x + size.width/2;
    pointsXAxis[2].y = y + size.height/2;
    pointsXAxis[3].x = x + size.width/2;
    pointsXAxis[3].y = y - size.height/2;
}

// Function to rotate a point around the center of the robot.
void Robot::rotatePoint(double angle){

    for(int i = 0; i < 4; i++){

        float temp_x = pointsXAxis[i].x - x;
        float temp_y = pointsXAxis[i].y - y;

        pointsXAxis[i].x = temp_x * cos(angle) - temp_y * sin(angle);
        pointsXAxis[i].y = temp_x * sin(angle) + temp_y * cos(angle);

        pointsXAxis[i].x += x;
        pointsXAxis[i].y += y;
    }
}

Wall::Wall(int x_, int y_, Size size_){
    x = x_;
    y = y_;
    size = size_;
    center_x = size.width/2;
    center_y = size.height/2;
    type = Wall::typeBackground;
}

Wall::Wall(Point p, Size size_){
    x = p.x;
    y = p.y;
    size = size_;
    center_x = size.width/2;
    center_y = size.height/2;
    type = Wall::typeBackground;
}

Wall::Wall(Point p, Size size_, squareType type_){
    x = p.x;
    y = p.y;
    size = size_;
    center_x = size.width/2;
    center_y = size.height/2;
    type = type_;
}

int Wall::getX(){
    return x;
}

int Wall::getY(){
    return y;
}

void Wall::setType(squareType type_){
    type = type_;
}

void Wall::setSize(Size size_){
    size = size_;
}

Wall::squareType Wall::getType(){
    return type;
}

bool Wall::contains(int x_, int y_){

    float x1, x2, y1, y2;

    x1 = x_;
    x2 = x_ + size.width;
    y1 = y_;
    y2 = y_ + size.height;

    if(((x_ >= x) && (x_ <= x + size.width)) && ((y_ >= y) && (y_ <= y + size.height))){
        return true;
    }
    else if(((x1 > x) && (x1 < x + size.width)) && (((y1 > y) && (y1 < y + size.height)) || ((y2 > y) && (y2 < y + size.height)))){
        return true;
    }
    else if(((x2 > x) && (x2 < x + size.width)) && (((y1 > y) && (y1 < y + size.height)) || ((y2 > y) && (y2 < y + size.height)))){
        return true;
    }
    else{
        return false;
    }
}

bool Wall::contains(int x_, int y_, int tolerance){

    if(((x_ - tolerance >= x) && (x_ + tolerance <= x + size.width)) && ((y_ - tolerance >= y) && (y_ + tolerance <= y + size.height))){
        return true;
    }
    else{
        return false;
    }
}
