#include "GUIWindow.h"
#include <iostream>
#include <cmath>

GUI::GUI(){
    //Setting variables
    emptySize = Size(1,1);
    wallSize = Size(8,8);
    robotSize = Size(round(10 * scale_factor / 100), round(15 * scale_factor / 100));

    //Create display
    display = XOpenDisplay(NULL);

    //Check if display is open
    if (display == NULL) {
        std::cerr << "Unable to open X display" << std::endl;
        exit(0);
    }

    //Format
    screen = DefaultScreen(display);
    window = XCreateSimpleWindow(display, RootWindow(display, screen), 10, 10, XDisplayWidth(display, screen), XDisplayHeight(display, screen), 1, BlackPixel(display, screen), WhitePixel(display, screen));
    XSelectInput(display, window, ExposureMask);

    gcWall = XCreateGC(display, window, 0, NULL);
    gcEmpty = XCreateGC(display, window, 0, NULL);
    gcRobot = XCreateGC(display, window, 0, NULL);
    gcRed = XCreateGC(display, window, 0, NULL);
    gcGreen = XCreateGC(display, window, 0, NULL);

    //Color
    Colormap colorMap = DefaultColormap(display, screen);
    XColor  color_Robot, color_Red, color_Green;

    XParseColor(display, colorMap, "steelblue", &color_Robot);
    XAllocColor(display, colorMap, &color_Robot);

    XParseColor(display, colorMap, "firebrick3", &color_Red);
    XAllocColor(display, colorMap, &color_Red);

    XParseColor(display, colorMap, "springgreen4", &color_Green);
    XAllocColor(display, colorMap, &color_Green);

    XSetForeground(display, gcWall, BlackPixel(display, screen)); //Wall graphics
    XSetForeground(display, gcRobot, color_Robot.pixel); //Robot graphics
    XSetForeground(display, gcRed, color_Red.pixel); //Orient X
    XSetForeground(display, gcGreen, color_Green.pixel); //Orient Y

    //Robot initialization
    robot = Robot(Point(XDisplayWidth(display, screen)/2, XDisplayHeight(display, screen)-200), robotSize);

    wallFrags.push_back(Wall(Point(0,0), emptySize));
}

using namespace std;

bool GUI::spaceFree(int x_, int y_){
    for(std::size_t i = 0; i < wallFrags.size(); i++){
        if(wallFrags[i].contains(x_, y_)){
            return false;
        }
        else{
            continue;
        }
    }
    return true;
}

void GUI::lidarReading(float angle, float len){
    float x_com, y_com;

    x_com = len * cos(angle) * scale_factor;
    y_com = len * sin(angle) * scale_factor;

    if(spaceFree(robot.x + x_com, robot.y + y_com)){ //if there is no wall drawn at the detected point, create a new wall.
        wallFrags.push_back(Wall(Point(robot.x + x_com, robot.y + y_com), wallSize, Wall::typeWall));
    }

    // Check if wallFrags has more than 1000 entries and remove until it is below:
    /*
    while(wallFrags.size() > 400){
        wallFrags.erase(wallFrags.begin());
    }
    */

}

void GUI::movementRobot(float robot_x, float robot_y){
    // Update robot position to global coordinates
    robot.x = robot.start_x + robot_x * scale_factor;
    robot.y = robot.start_y + robot_y * scale_factor;
}

void GUI::rescale() {
    //if the robot comes near the edge, move it, and the walls.
    if(robot.x < 400){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].x = wallFrags[i].x + 400;
        }

        robot.start_x = robot.start_x + 400;
    }
    else if(robot.x > (XDisplayWidth(display, screen) - 400)){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].x = wallFrags[i].x - 400;
        }

        robot.start_x = robot.start_x - 400;
    }
    else if(robot.y < 400){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].y = wallFrags[i].y + 400;
        }

        robot.start_y = robot.start_y + 400;
    }
    else if(robot.y > (XDisplayHeight(display, screen) - 400)){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].y = wallFrags[i].y - 400;
        }

        robot.start_y = robot.start_y - 400;
    }
}

void GUI::paintMap(){
    //Draws the wallFrags
    for(int i = 0; i < wallFrags.size(); i++){
        XFillRectangle(display, window, gcWall, wallFrags[i].x - wallFrags[i].size.width/2, wallFrags[i].y - wallFrags[i].size.height/2, wallFrags[i].size.width, wallFrags[i].size.height);
    }
}

void GUI::findPoints(vector<Point>& points, Point p1, Point p2, bool TF){
    if(TF){
        points.clear();
    }

    points.push_back(p1);

    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;
    int dist = sqrt(((p2.x - p1.x)*(p2.x - p1.x)) + ((p2.y - p1.y)*(p2.y - p1.y)));

    for(int i = 1; i < (dist/4); i++){
        points.push_back(Point(p1.x + (dx * i)/(dist/4), p1.y + (dy * i)/(dist/4)));
    }
}

void GUI::drawRect(Point vertices[4], GC gc){ //Assumes the square is rectangular (corner angles = 90 deg), but can be rotated.

    for(int i = 0; i < 4; i++){ //Draw the walls of the rectangle
        if(i < 3){
            XDrawLine(display, window, gc, vertices[i].x, vertices[i].y, vertices[i+1].x, vertices[i+1].y);
        }
        else{
            XDrawLine(display, window, gc, vertices[i].x, vertices[i].y, vertices[0].x, vertices[0].y);
        }
    }

    vector<Point> pointsStart; //Points between two vertices
    vector<Point> pointsEnd; //Points between the other two vertices

    findPoints(pointsStart, vertices[0], vertices[3], 4);
    findPoints(pointsEnd, vertices[1], vertices[2], 4);

    for(int i = 0; i < pointsStart.size(); i++){
        XDrawLine(display, window, gc, pointsStart[i].x, pointsStart[i].y, pointsEnd[i].x, pointsEnd[i].y);
    }

    findPoints(pointsStart, vertices[0], vertices[1], 4);
    findPoints(pointsEnd, vertices[3], vertices[2], 4);

    for(int i = 0; i < pointsStart.size(); i++){
        XDrawLine(display, window, gc, pointsStart[i].x, pointsStart[i].y, pointsEnd[i].x, pointsEnd[i].y);
    }
}

void GUI::paintRobot(float yaw) {
    robot.rotate(yaw);

    drawRect(robot.locRobot, gcRobot);
    drawRect(robot.orientXRobot, gcRed);
    drawRect(robot.orientYRobot, gcGreen);

    XPutBackEvent(display, &event);
}

void GUI::update(bool& update, float angle, float len, float robot_x, float robot_y, float robot_yaw){
    robot.update();
    paintRobot(robot_yaw);
    movementRobot(robot_x, robot_y);
    rescale();
    lidarReading(angle, len);
    paintMap();

    if(XPending(display) > 0){
        XNextEvent(display, &event);
        if(event.type == Expose){
            update = true;
        }
    }

    if(update){
        XSync(display, False);
        XClearWindow(display, window);
        update = false;
    }
}

void GUI::show(){
    XMapWindow(display, window);
    XNextEvent(display, &event);
}
