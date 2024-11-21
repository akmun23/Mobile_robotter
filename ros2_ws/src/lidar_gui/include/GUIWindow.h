#ifndef GUIWINDOW_H
#define GUIWINDOW_H

#include <X11/Xlib.h>
#include <vector>
#include <unistd.h>
#include <cmath>
#include <mutex>
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

    //Robot
    Robot robot;

    int update_counter = 0;
    const float scale_factor = 320.0;

public:
    GUI();
    bool spaceFree(int x_, int y_);
    int squareOccupy(int x_, int y_);
    void movementRobot(float robot_x, float robot_y);
    void lidarReading(float angle, float len, float yaw);
    void rescale();
    void paintMap();
    void paintRobot(float yaw);
    void update(bool& update, float angle, float len, float robot_x, float robot_y, float robot_yaw);
    void show();
};

#endif // GUIWINDOW_H
