#ifndef GUIWINDOW_H
#define GUIWINDOW_H

#include <X11/Xlib.h>
#include <vector>
#include <unistd.h>
#include <cmath>
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

public:
    GUI();
    bool spaceFree(int x_, int y_);
    void movementRobot(float angle, float intensity);
    void lidarReading(float angle, float len);
    void rescale();
    void paintMap();
    void paintRobot();
    void update(bool& update, float angle, float len);
    void show();
};

#endif // GUIWINDOW_H