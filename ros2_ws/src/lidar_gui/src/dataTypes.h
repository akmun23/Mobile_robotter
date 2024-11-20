#ifndef DATATYPES_H
#define DATATYPES_H

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

//Point
struct Point{

    Point(int x_, int y_){
        x = x_;
        y = y_;
    }

    int x, y;
};

//Size
struct Size{
    int width, height;

    Size(){}

    Size(int width_, int height_){
        width = width_;
        height = height_;
    }
};

//Robot
struct Robot{
    float x, y, orient;
    Size size;

    Robot(){}

    Robot(int x_, int y_, Size size_){
        x = x_;
        y = y_;
        size = size_;
    }

    Robot(Point p, Size size_){
        x = p.x;
        y = p.y;
        size = size_;
    }
};

//Wall
class Wall{

public:
    enum squareType{
        typeEmpty,
        typeBackground,
        typeWall
    };

private:
    squareType type;

public:
    int x, y, center_x, center_y;
    Size size;

    Wall(int x_, int y_, Size size_){
        x = x_;
        y = y_;
        size = size_;
        center_x = size.width/2;
        center_y = size.height/2;
        type = Wall::typeBackground;
    }

    Wall(Point p, Size size_){
        x = p.x;
        y = p.y;
        size = size_;
        center_x = size.width/2;
        center_y = size.height/2;
        type = Wall::typeBackground;
    }

    Wall(Point p, Size size_, squareType type_){
        x = p.x;
        y = p.y;
        size = size_;
        center_x = size.width/2;
        center_y = size.height/2;
        type = type_;
    }

    void setType(squareType type_){
        type = type_;
    }

    squareType getType(){
        return type;
    }

    int getX(){
        return x;
    }

    int getY(){
        return y;
    }

    bool contains(int x_, int y_){

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

    bool contains(int x_, int y_, int tolerance){

        if(((x_ - tolerance >= x) && (x_ + tolerance <= x + size.width)) && ((y_ - tolerance >= y) && (y_ + tolerance <= y + size.height))){
            return true;
        }
        else{
            return false;
        }
    }
};


#endif // DATATYPES_H
