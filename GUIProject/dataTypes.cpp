#include "dataTypes.h"

Point::Point(int x_, int y_){
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
