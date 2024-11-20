#ifndef DATATYPES_H
#define DATATYPES_H

#include <vector>
#include <cmath>

using namespace std;

//Point
struct Point{
    int x, y;

    Point(int x_, int y_);
};

//Size
struct Size{
    int width, height;

    Size(){}

    Size(int width_, int height_);
};

//Robot
struct Robot{
    float x, y, orient;
    Size size;

    Robot(){}

    Robot(int x_, int y_, Size size_);

    Robot(Point p, Size size_);
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

    Wall(int x_, int y_, Size size_);

    Wall(Point p, Size size_);

    Wall(Point p, Size size_, squareType type_);

    void setType(squareType type_);
    void setSize(Size size_);

    squareType getType();

    int getX();
    int getY();

    bool contains(int x_, int y_);
    bool contains(int x_, int y_, int tolerance);
};


#endif // DATATYPES_H
