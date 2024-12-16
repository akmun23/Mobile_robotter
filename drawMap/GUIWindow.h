#ifndef GUIWINDOW_H
#define GUIWINDOW_H

#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlError>
#include <QDebug>

#include <X11/Xlib.h>
#include <vector>
#include <unistd.h>
#include <cmath>

#include "dataTypes.h"

#undef slots
#undef signals

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

    const float scale_factor = 320.0;

    QSqlDatabase db;

public:
    GUI();
    ~GUI();
    bool spaceFree(int x_, int y_);
    void movementRobot(float robot_x, float robot_y);
    void lidarReading(float angle, float len);
    void rescale();
    void paintMap();
    void findPoints(std::vector<Point>& points, Point p1, Point p2, bool TF = true);
    void drawRect(Point vertices[4], GC gc);
    void paintRobot(float yaw);
    void update(bool& update);
    void show();
};

#endif // GUIWINDOW_H
