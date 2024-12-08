#ifndef GUIWINDOW_H
#define GUIWINDOW_H

#include <vector>
#include <unistd.h>
#include <math.h>
#include <cmath>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlError>
#include <QDebug>

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
    std::vector<Wall> robotFrags;

    //Graphics - "Colors"
    GC gcWall;
    GC gcEmpty;
    GC gcRobot;
    GC gcRed;
    GC gcGreen;

    const float scale_factor = 160.0;

    QSqlDatabase db;

public:
    GUI();
    ~GUI();
    bool spaceFree(int x_, int y_);
    void lidarReading(double angle, double len, double robot_x, double robot_y, double robot_yaw);
    void paintMap();
    void paintRobot(double robot_x, double robot_y);
    void robotPaint();
    void update(bool& update);
    void show();
};

#endif // GUIWINDOW_H
