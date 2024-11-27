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

Robot::Robot(Point p, Size size_){
    x = p.x;
    y = p.y;
    angle = 0;

    size = size_;

    //Robot location visual
    //locRobot[0].x = x - size.width/2;
    //locRobot[0].y = y - size.height/2;
    //locRobot[1].x = x - size.width/2;
    //locRobot[1].y = y + size.height/2;
    //locRobot[2].x = x + size.width/2;
    //locRobot[2].y = y + size.height/2;
    //locRobot[3].x = x + size.width/2;
    //locRobot[3].y = y - size.height/2;

    locRobot[0].x = x - size.width/2;
    locRobot[0].y = y - size.height/2;
    locRobot[1].x = x + size.width/2;
    locRobot[1].y = y - size.height/2;
    locRobot[2].x = x + size.width/2;
    locRobot[2].y = y + size.height/2;
    locRobot[3].x = x - size.width/2;
    locRobot[3].y = y + size.height/2;

    //Robot X Orient Axis
    orientXRobot[0].x = x + size.width/2;
    orientXRobot[0].y = y - size.height/6;
    orientXRobot[1].x = x + (2*size.width);
    orientXRobot[1].y = y - size.height/6;
    orientXRobot[2].x = x + (2*size.width);
    orientXRobot[2].y = y + size.height/6;
    orientXRobot[3].x = x + size.width/2;
    orientXRobot[3].y = y + size.height/6;


    //Robot Y Orient Axis
    orientYRobot[0].x = x - size.width/6;
    orientYRobot[0].y = y + size.height/2;
    orientYRobot[1].x = x - size.width/6;
    orientYRobot[1].y = y + (2*size.height);
    orientYRobot[2].x = x + size.width/6;
    orientYRobot[2].y = y + (2*size.height);
    orientYRobot[3].x = x + size.width/6;
    orientYRobot[3].y = y + size.height/2;
}

// Function to rotate a point around the center of the robot.
void Robot::rotateToAngle(double angle_){

    float temp_x = 0, temp_y = 0, temp_angle = 0;

    temp_angle = angle_ - angle; //Calculate the difference in angle
    angle = angle_; //set "angle_" as the new angle, which assumes we will roatet to that angle

    for(int i = 0; i < 4; i++){

        temp_x = locRobot[i].x - x;
        temp_y = locRobot[i].y - y;

        locRobot[i].x = temp_x * cos(temp_angle) - temp_y * sin(temp_angle) + x;
        locRobot[i].y = temp_x * sin(temp_angle) + temp_y * cos(temp_angle) + y;

        temp_x = orientXRobot[i].x - x;
        temp_y = orientXRobot[i].y - y;

        orientXRobot[i].x = temp_x * cos(temp_angle) - temp_y * sin(temp_angle) + x;
        orientXRobot[i].y = temp_x * sin(temp_angle) + temp_y * cos(temp_angle) + y;

        temp_x = orientYRobot[i].x - x;
        temp_y = orientYRobot[i].y - y;

        orientYRobot[i].x = temp_x * cos(temp_angle) - temp_y * sin(temp_angle) + x;
        orientYRobot[i].y = temp_x * sin(temp_angle) + temp_y * cos(temp_angle) + y;
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
