#include "GUIWindow.h"
#include <iostream>
#include <cmath>
#include <algorithm>

GUI::GUI(){
    //Setting variables
    emptySize = Size(1,1);
    wallSize = Size(8,8);

    robotSize = Size(20,20);

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

    wallFrags.push_back(Wall(Point(0,0), emptySize));
}

bool GUI::spaceFree(int x_, int y_){
    for(int i = 0; i < wallFrags.size(); i++){
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
    for(int i = 0; i < wallFrags.size(); i++){
        if(wallFrags[i].contains(x_, y_)){
            return i;
        }
        else{
            continue;
        }
    }

    return 0;
}

void GUI::lidarReading(float angle, float len){
    float x_com, y_com;

    x_com = len*cos(angle);
    y_com = len*sin(angle);

    for(int i = 0; i < wallFrags.size(); i++){

        //Generate wall
        if((i == (wallFrags.size() - 1)) && (spaceFree(robot.x + x_com, robot.y + y_com))){
            wallFrags.push_back(Wall(Point(robot.x + x_com, robot.y + y_com), wallSize, Wall::typeWall)); //
            break;
        }
    }
}

void GUI::movementRobot(float angle, float intensity){

    int x_com, y_com;

    x_com = intensity * cos(angle);
    y_com = intensity * sin(angle);

    robot.x = robot.x + x_com;
    robot.y = robot.y + y_com;
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

void GUI::fillRect(Point vertices[4], GC gc){

    float x_min = std::min({vertices[0].x, vertices[1].x, vertices[2].x, vertices[3].x});
    float x_max = std::max({vertices[0].x, vertices[1].x, vertices[2].x, vertices[3].x});

    int x_diff = static_cast<int>(x_max - x_min);

    //float y_min = std::min({vertices[0].y, vertices[1].y, vertices[2].y, vertices[3].y});
    //float y_max = std::max({vertices[0].y, vertices[1].y, vertices[2].y, vertices[3].y});

    float y0_corner = vertices[0].y, y1_corner = vertices[1].y, y3_corner = vertices[3].y, y_upper_diff = y0_corner - y1_corner;

    cout << "------" << endl << "y1: " << vertices[1].y << " y2: " << vertices[3].y << endl;

    Point Points[x_diff * 2];

    for(int i = x_min; i < x_max; i++){

        cout << "y1: " << y0_corner + ((y_upper_diff/x_diff)*i) << " y2: " << y3_corner + ((y_upper_diff/x_diff)*i) << endl;

        XDrawLine(display, window, gc, x_min + i, y0_corner + ((y_upper_diff/x_diff)*i), x_min + i, y3_corner + ((y_upper_diff/x_diff)*i));

        //Points[i] = Point(x_min + i, y0_corner + (y_upper_diff/x_diff)*i);
        //Points[i] = Point(x_min + i, y3_corner + (y_upper_diff/x_diff)*i);
    }

    //XDrawLines(display, window, gc, Points);


}

void GUI::paintMap(){
    //Draw the rectangles the correspoding color
    for(int i = 0; i < wallFrags.size(); i++){
        XFillRectangle(display, window, (wallFrags[i].getType() == Wall::typeWall ? gcWall : gcEmpty), wallFrags[i].x - wallFrags[i].size.width/2, wallFrags[i].y - wallFrags[i].size.height/2, wallFrags[i].size.width, wallFrags[i].size.height);
    }
}

void GUI::paintRobot(){
    usleep(1000);
    robot.rotatePoint(0);

    for(int i = 0; i < 4; i++){

        if(i < 3){
            XDrawLine(display, window, gcRobot, robot.locRobot[i].x, robot.locRobot[i].y, robot.locRobot[i+1].x, robot.locRobot[i+1].y);
            XDrawLine(display, window, gcRed, robot.orientXRobot[i].x, robot.orientXRobot[i].y, robot.orientXRobot[i+1].x, robot.orientXRobot[i+1].y);
            XDrawLine(display, window, gcGreen, robot.orientYRobot[i].x, robot.orientYRobot[i].y, robot.orientYRobot[i+1].x, robot.orientYRobot[i+1].y);
        }
        else{
            XDrawLine(display, window, gcRobot, robot.locRobot[i].x, robot.locRobot[i].y, robot.locRobot[0].x, robot.locRobot[0].y);
            XDrawLine(display, window, gcRed, robot.orientXRobot[i].x, robot.orientXRobot[i].y, robot.orientXRobot[0].x, robot.orientXRobot[0].y);
            XDrawLine(display, window, gcGreen, robot.orientYRobot[i].x, robot.orientYRobot[i].y, robot.orientYRobot[0].x, robot.orientYRobot[0].y);
        }
    }

    fillRect(robot.locRobot, gcRobot);

    /*int minY = std::min({robot.pointsXAxis[0].y, robot.pointsXAxis[1].y, robot.pointsXAxis[2].y, robot.pointsXAxis[3].y});
    int maxY = std::max({robot.pointsXAxis[0].y, robot.pointsXAxis[1].y, robot.pointsXAxis[2].y, robot.pointsXAxis[3].y});
    vector<float> intersections;


    auto intersection = [y](int )*/


    /*auto intersect = [y](int x1, int y1, int x2, int y2) -> int{
    if (y1 == y2) return x1; // Horizontal edge
    return static_cast<int>(x1 + (y - y1) * (x2 - x1) / static_cast<float>(y2 - y1));
    };*/

    XPutBackEvent(display, &event);
}

void GUI::update(bool& update){
    int update_counter = 0;

    while(1){

        lidarReading(rand()%360, 200); //Make a timer function

        //movementRobot(1, 1);

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
            XSync(display, False);
            XClearWindow(display, window);
            update = false;
        }
    }
}

void GUI::show(){
    XMapWindow(display, window);
    XNextEvent(display, &event);
}
