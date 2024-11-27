#ifndef GUIWINDOW_H
#define GUIWINDOW_H

#include <vector>
#include <unistd.h>
#include <math.h>
#include <cmath>

#include "dataTypes.h"

class GUI{
private:
    //Screen
    Display *display;
    Window window;
    XEvent event;
    int screen;

    //Structure
    Size emptySize;
    Size wallSize;
    Size robotSize;
    std::vector<Wall> wallFrags;

    //Graphics - "Colors"
    GC gcWall;
    GC gcEmpty;
    GC gcRobot;
    GC gcRed;
    GC gcGreen;

    //Robot
    Robot robot;

public:
    GUI();
    bool spaceFree(int x_, int y_);
    int squareOccupy(int x_, int y_);
    void movementRobot(float angle, float intensity);
    void lidarReading(float angle, float len);
    void rescale();
    void findPoints(vector<Point>& points, Point p1, Point p2, bool TF);
    void drawRect(Point vertices[], GC gc);
    void paintMap();
    void paintRobot();
    void update(bool& update);
    void show();
};

#endif // GUIWINDOW_H
