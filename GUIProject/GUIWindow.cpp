#include "GUIWindow.h"
#include <iostream>
#include <cmath>

GUI::GUI(){

    //Setting variables
    emptySize = Size(1,1);
    wallSize = Size(8,8);

    robotSize = Size(30,30);

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

    //Create graphics contexts
    gcWall = XCreateGC(display, window, 0, NULL);
    gcEmpty = XCreateGC(display, window, 0, NULL);
    gcRobot = XCreateGC(display, window, 0, NULL);
    gcRed = XCreateGC(display, window, 0, NULL);
    gcGreen = XCreateGC(display, window, 0, NULL);

    //Color
    Colormap colorMap = DefaultColormap(display, screen);
    XColor color_Wall, color_Empty, color_Robot, color_Red, color_Green;

    XParseColor(display, colorMap, "snow4", &color_Empty);
    XAllocColor(display, colorMap, &color_Empty);

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
    robot = Robot(Point(XDisplayWidth(display, screen)/2, XDisplayHeight(display, screen)/2), robotSize);

    //Create single wall to be able to iterate through the vector.
    wallFrags.push_back(Wall(Point(0,0), emptySize));
}

bool GUI::spaceFree(int x_, int y_, float tolerance = 0){ //Returns true if no wallFrags was found to contain the given point
    for(int i = 0; i < wallFrags.size(); i++){
        if(wallFrags[i].contains(x_, y_, tolerance)){
            return false;
        }
    }

    return true;
}

void GUI::lidarReading(float angle, float len){
    float x_com, y_com;

    //Calculate the x and y components
    x_com = len*cos(angle);
    y_com = len*sin(angle);

    if(spaceFree(robot.x + x_com, robot.y + y_com)){ //if there is no wall drawn at the detected point, create a new wall.
        wallFrags.push_back(Wall(Point(robot.x + x_com, robot.y + y_com), wallSize, Wall::typeWall));
    }
}

void GUI::movementRobot(float angle, float intensity){

    float x_com, y_com;

    //calculate the components
    x_com = intensity * cos(angle);
    y_com = intensity * sin(angle);

    //add movements to the robots position
    robot.x += x_com;
    robot.y += y_com;

    //add movement to visualsr
    robot.update(x_com, y_com);

    //subtract the robots movement from the position of the walls. Makes the walls stay in place, in relation to the robot.
    for(int i = 0; i < wallFrags.size(); i++){
        wallFrags[i].x -= x_com;
        wallFrags[i].y -= y_com;
    }
}

void GUI::rescale(){

    //if the robot comes near the edge, move it, and the walls.
    if(robot.x < 400){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].x = wallFrags[i].x + 400;
        }

        robot.x = robot.x + 400;
    }
    else if(robot.x > (XDisplayWidth(display, screen) - 400)){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].x = wallFrags[i].x - 400;
        }

        robot.x = robot.x - 400;
    }
    else if(robot.y < 400){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].y = wallFrags[i].y + 400;
        }

        robot.y = robot.y + 400;
    }
    else if(robot.y > (XDisplayHeight(display, screen) - 400)){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].y = wallFrags[i].y - 400;
        }

        robot.y = robot.y - 400;
    }
}

void GUI::findPoints(vector<Point>& points, Point p1, Point p2, int tolerance = 1, bool TF = true){ //add a point for every "tolerance" amount of points, to "points"
    if(TF){
        points.clear();
    }

    //calculate the vector and distance between points.
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;
    int dist = sqrt(((p2.x - p1.x)*(p2.x - p1.x)) + ((p2.y - p1.y)*(p2.y - p1.y)));

    //make a point for every "tolerance" amount of points, and add to vector.
    for(int i = 1; i < (dist/tolerance); i++){
        points.push_back(Point(p1.x + (dx * i)/(dist/tolerance), p1.y + (dy * i)/(dist/tolerance)));
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

void GUI::paintMap(){
    //Draws the wallFrags
    for(int i = 0; i < wallFrags.size(); i++){
        XFillRectangle(display, window, gcWall, wallFrags[i].x - wallFrags[i].size.width/2, wallFrags[i].y - wallFrags[i].size.height/2, wallFrags[i].size.width, wallFrags[i].size.height);
    }
}

void GUI::paintRobot(){
    drawRect(robot.locRobot, gcRobot);
    drawRect(robot.orientXRobot, gcRed);
    drawRect(robot.orientYRobot, gcGreen);

    XPutBackEvent(display, &event);
}

void GUI::update(bool& update){

    while(1){

        usleep(10000);

        lidarReading(rand()%360, 200);

        movementRobot(0, 0.001); //Handles change in robot location
        rescale(); //Moves everything on the screen, to make the screen "wider", to compensate for limited screen size.

        robot.rotate(0.1);

        paintMap();
        paintRobot();

        if(XPending(display) > 0){ //Is there an Event waiting to be drawn?
            XNextEvent(display, &event); //Select the next event to be drawn.
            if(event.type == Expose){
                XSync(display, False); //Sync the screen with the data in the system
                XClearWindow(display, window); //Clear the window, needs to be after XSync, for some reason.
            }
        }
    }
}

void GUI::show(){
    XMapWindow(display, window);
    XNextEvent(display, &event);
}
