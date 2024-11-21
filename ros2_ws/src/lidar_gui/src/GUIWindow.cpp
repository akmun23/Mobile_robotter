#include "GUIWindow.h"
#include <iostream>
#include <cmath>

GUI::GUI(){
    //Setting variables
    emptySize = Size(5,5);
    wallSize = Size(5,5);
    robotSize = Size(15 * scale_factor / 100, 10 * scale_factor / 100);

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

    //Color
    Colormap colorMap = DefaultColormap(display, screen);
    XColor color_Robot;

    XParseColor(display, colorMap, "firebrick3", &color_Robot);
    XAllocColor(display, colorMap, &color_Robot);

    XSetForeground(display, gcWall, BlackPixel(display, screen)); //Wall graphics
    XSetForeground(display, gcRobot, color_Robot.pixel); //Robot graphics

    //Robot initialization
    robot = Robot(Point(XDisplayWidth(display, screen)/2, XDisplayHeight(display, screen)/2), robotSize);

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

int GUI::squareOccupy(int x_, int y_){
    for(std::size_t i = 0; i < wallFrags.size(); i++){
        if(wallFrags[i].contains(x_, y_)){
            return i;
        }
        else{
            continue;
        }
    }
    return 0;
}

void GUI::lidarReading(float angle, float len, float yaw){
    int x_com, y_com;
    float rotated_angle = angle + yaw;
    double local_x = len * cos(rotated_angle) * scale_factor;
    double local_y = len * sin(rotated_angle) * scale_factor;

    x_com = local_x;
    y_com = local_y;

    for(std::size_t i = 0; i < wallFrags.size(); i++){
        // Generate wallrobot.y +
        if(wallFrags[i].contains(robot.x + x_com, robot.y + y_com)){
            break;
        }
        else if((i == (wallFrags.size() - 1))){
            wallFrags.push_back(Wall(Point(robot.x + x_com, robot.y + y_com), wallSize, Wall::typeWall)); 
            break;
        }
    }

    // Check if wallFrags has more than 1000 entries and remove until it is below:
    while(wallFrags.size() > 1000){
        wallFrags.erase(wallFrags.begin());
    }
}

void GUI::movementRobot(float robot_x, float robot_y){
    // Update robot position to global coordinates
    robot.x = robot.start_x + robot_y * scale_factor;
    robot.y = robot.start_y + robot_x * scale_factor;
}

void GUI::rescale() {
    const int offset = 500;
    if (robot.x < (XDisplayWidth(display, screen) * 0.1)) {
        for (auto& wall : wallFrags) {
            wall.x += offset;
        }
        robot.x += offset;
    } else if (robot.x > (XDisplayWidth(display, screen) * 0.9)) {
        for (auto& wall : wallFrags) {
            wall.x -= offset;
        }
        robot.x -= offset;
    } else if (robot.y < (XDisplayHeight(display, screen) * 0.1)) {
        for (auto& wall : wallFrags) {
            wall.y += offset;
        }
        robot.y += offset;
    } else if (robot.y > (XDisplayHeight(display, screen) * 0.9)) {
        for (auto& wall : wallFrags) {
            wall.y -= offset;
        }
        robot.y -= offset;
    }
}

void GUI::paintMap(){
    //Draw the rectangles the correspoding color
    for(std::size_t i = 0; i < wallFrags.size(); i++){
        XFillRectangle(display, window, (wallFrags[i].getType() == Wall::typeWall ? gcWall : gcEmpty), wallFrags[i].x - wallFrags[i].size.width/2, wallFrags[i].y - wallFrags[i].size.height/2, wallFrags[i].size.width, wallFrags[i].size.height);
    }
}

void GUI::paintRobot(float yaw) {
    // Draw the robot as a square
    XFillRectangle(display, window, gcRobot, robot.x - robot.size.width/2, robot.y - robot.size.height/2, robot.size.width, robot.size.height);
}

void GUI::update(bool& update, float angle, float len, float robot_x, float robot_y, float robot_yaw){
    movementRobot(robot_x, robot_y);
    lidarReading(angle, len, robot_yaw);

    if((robot.x < (XDisplayWidth(display, screen) * 0.1)) || (robot.x > (XDisplayWidth(display, screen) * 0.9)) || (robot.y < (XDisplayHeight(display, screen) * 0.1)) || (robot.y > (XDisplayWidth(display, screen) * 0.9))){
        cout << "entered rescale" << endl;
        rescale();
    }

    paintMap();
    paintRobot(robot_yaw);

    if(XPending(display) > 0){
        XNextEvent(display, &event);
        if(event.type == Expose){
            update = true;
        }
    }

    if (update_counter > 500) {
        XClearWindow(display, window);
        update_counter = 0;
    }
    update_counter++;
}

void GUI::show(){
    XMapWindow(display, window);
    XNextEvent(display, &event);
}
