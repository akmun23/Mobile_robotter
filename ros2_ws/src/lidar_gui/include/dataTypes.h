#ifndef DATATYPES_H
#define DATATYPES_H

#include <vector>
#include <cmath>

using namespace std;

//Point
struct Point{
    int x, y;

    Point(){}

    Point(float x_, float y_);
};

//Size
struct Size{
    int width, height;

    Size(){}

    Size(int width_, int height_);
};

//Robot
struct Robot{
    double_t x, y, start_x, start_y, angle;
    Size size, xAxis;
    Point locRobot[4], orientXRobot[4], orientYRobot[4];

    Robot(){}

    Robot(Point p, Size size_);

    void rotate(double angle);
    void update();
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

    squareType getType() const;

    int getX();
    int getY();

    bool contains(int x_, int y_);

    bool contains(int x_, int y_, int tolerance);
};

#endif // DATATYPES_H
