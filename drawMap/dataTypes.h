#ifndef DATATYPES_H
#define DATATYPES_H

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <vector>
#include <cmath>

using namespace std;

//Point
struct Point{
    float x, y;

    Point(){}

    Point(int x_, int y_);

    Point(float x_, float y_);
};

//Size
struct Size{
    int width, height;

    Size(){}

    Size(int width_, int height_);
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
