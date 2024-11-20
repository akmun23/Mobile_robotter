#include "GUIWindow.h"
#include <iostream>
#include <cmath>

GUI::GUI(){

    //Setting variables
    emptySize = Size(1,1);
    wallSize = Size(8,8);

    robotSize = Size(10,10);

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
    XColor color_Wall, color_Empty, color_Robot;

    //XParseColor(display, colorMap, "gray50", &color_Wall);
    //XAllocColor(display, colorMap, &color_Wall);

    XParseColor(display, colorMap, "snow4", &color_Empty);
    XAllocColor(display, colorMap, &color_Empty);

    XParseColor(display, colorMap, "firebrick3", &color_Robot);
    XAllocColor(display, colorMap, &color_Robot);

    XSetForeground(display, gcWall, BlackPixel(display, screen)); //Wall graphics
    XSetForeground(display, gcEmpty, color_Empty.pixel); //Empty graphics
    XSetForeground(display, gcRobot, color_Robot.pixel); //Robot graphics

    //Robot initialization
    robot = Robot(Point(XDisplayWidth(display, screen)/2, XDisplayHeight(display, screen)/2), Size(10,10));

    wallFrags.push_back(Wall(Point(0,0), emptySize));
}

using namespace std;

bool GUI::spaceFree(int x_, int y_){
    for(int i = 0; i < wallFrags.size(); i++){
        if(wallFrags[i].contains(x_, y_)){
            return false;
        }
        else{
            continue;
        }
    }

    //cout << "spacefree returned true;" << endl;
    return true;
}

void GUI::lidarReading(float angle, float len){
    float x_com, y_com;

    angle = angle*(180/M_PI);

    x_com = len*cos(angle);
    y_com = len*sin(angle);

    for(int i = 0; i < wallFrags.size(); i++){

        //Generate wall
        if(wallFrags[i].contains(robot.x + x_com, robot.y + y_com) && (wallFrags[i].getType() != Wall::typeWall)){ //Does a square exist on the recorded coords?

            //if a wall has been recorded, its type should be changed
            wallFrags[i].setType(Wall::typeWall);

            break;
        }
        else if((i == (wallFrags.size() - 1)) && (spaceFree(robot.x + x_com, robot.y + y_com))){
            wallFrags.push_back(Wall(Point(robot.x + x_com, robot.y + y_com), wallSize, Wall::typeWall)); //
            break;
        }
    }

    //if there is anything between the robot and wall, it should be changed to empty space
    //if there isnt anything between the robot and wall, an empty space should be made
    for(int i = 1; i < len * 0.93; i++){
        if(spaceFree(robot.x + (x_com/len)*i, robot.y + (y_com/len)*i)){
            wallFrags.push_back(Wall(Point(robot.x + (x_com/len)*i, robot.y + (y_com/len)*i), emptySize, Wall::typeEmpty));
        }
        else{
            //get rect that is in that place, and change size and type
            //cout << "space wasnt free" << endl;
        }
    }
}

void GUI::movementRobot(float angle, float intensity){

    int x_com, y_com;

    angle = angle*(180/M_PI);

    x_com = intensity * cos(angle);
    y_com = intensity * sin(angle);

    robot.x = robot.x + x_com;
    robot.y = robot.y + y_com;

    XPutBackEvent(display, &event);
}

void GUI::rescale(){

    if(robot.x < (XDisplayWidth(display, screen) * 0.1)){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].x = wallFrags[i].x + 500;
        }

        robot.x = robot.x + 500;
    }
    else if(robot.x > (XDisplayWidth(display, screen) * 0.9)){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].x = wallFrags[i].x - 500;
        }

        robot.x = robot.x - 500;
    }
    else if(robot.y < (XDisplayHeight(display, screen) * 0.1)){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].y = wallFrags[i].y + 500;
        }

        robot.y = robot.y + 500;
    }
    else{
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].y = wallFrags[i].y - 500;
        }

        robot.y = robot.y - 500;
    }
}

void GUI::paintMap(){
    //Draw the rectangles the correspoding color
    for(int i = 0; i < wallFrags.size(); i++){
        XFillRectangle(display, window, (wallFrags[i].getType() == Wall::typeWall ? gcWall : gcEmpty), wallFrags[i].x - wallFrags[i].size.width/2, wallFrags[i].y - wallFrags[i].size.height/2, wallFrags[i].size.width, wallFrags[i].size.height);
    }
}

void GUI::paintRobot(){
    XFillRectangle(display, window, gcRobot, robot.x - robot.size.width/2, robot.y - robot.size.height/2, robotSize.width, robotSize.height);
}

void GUI::update(bool& update){
    int move = 0;

    while(1){

        lidarReading(rand()%360, 200); //Make a timer function
        //movementRobot(1, 10);

        /*if((robot.x < (XDisplayWidth(display, screen) * 0.1)) || (robot.x > (XDisplayWidth(display, screen) * 0.9)) || (robot.y < (XDisplayHeight(display, screen) * 0.1)) || (robot.y > (XDisplayWidth(display, screen) * 0.9))){
            cout << "entered rescale" << endl;
            rescale();
        }*/

        paintMap(); //First detected walls gets drawn over, because of the hierachy of the vector. Squares should maybe also be constructed through a middle point, and not top left
        paintRobot();

        if(XPending(display) > 0){
            XNextEvent(display, &event);
            if(event.type == Expose){
                update = true;
            }
        }

        if(update){
            //XClearWindow(display, window);
            XSync(display, False);
            update = false;
        }

    }
}

void GUI::show(){
    XMapWindow(display, window);
    XNextEvent(display, &event);
}
