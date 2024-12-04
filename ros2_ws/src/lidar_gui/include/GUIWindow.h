#ifndef GUIWINDOW_H
#define GUIWINDOW_H

#include <X11/Xlib.h>
#include <vector>
#include <unistd.h>
#include <cmath>
#include <algorithm>
#include "dataTypes.h"

class GUI{
private:
    //Screen
    Display *display;
    Window window;
    XEvent event;
    int screen;

    //Values
    Size emptySize;
    Size wallSize;
    Size robotSize;
    std::vector<Wall> wallFrags;

    //Colors
    GC gcWall;
    GC gcEmpty;
    GC gcRobot;
    GC gcRed;
    GC gcGreen;

    //Robot
    Robot robot;

    int update_counter = 0;
    const float scale_factor = 320.0;
    float _prev_angle = 0;
    int counter = 0;

public:
    GUI();
    bool spaceFree(int x_, int y_);
    int squareOccupy(int x_, int y_);
    void movementRobot(float robot_x, float robot_y);
    void lidarReading(float angle, float len, float yaw);
    void rescale();
    void paintMap();
    void findPoints(vector<Point>& points, Point p1, Point p2, bool TF = true);
    void drawRect(Point vertices[4], GC gc);
    void paintRobot(float yaw);
    void update(bool& update, float angle, float len, float robot_x, float robot_y, float robot_yaw);
    void show();
};

#endif // GUIWINDOW_H
