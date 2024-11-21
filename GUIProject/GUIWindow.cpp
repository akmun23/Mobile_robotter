#include "GUIWindow.h"
#include <iostream>
#include <cmath>

GUI::GUI(){
    QSqlDatabase db = QSqlDatabase::addDatabase("QMYSQL");
    db.setHostName("localhost");       // Set the correct hostname
    db.setDatabaseName("lidar_db");    // Set your database name
    db.setUserName("aksel");           // Set the MySQL username
    db.setPassword("hua28rdc");        // Set the MySQL password

    if (!db.open()) {
        qDebug() << "Database error:" << db.lastError().text();
        return;
    }

    //Setting variables
    emptySize = Size(4, 4);
    wallSize = Size(8, 8);

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

    //Color
    Colormap colorMap = DefaultColormap(display, screen);
    XColor color_Wall, color_Empty, color_Robot;

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
}

void GUI::lidarReading(float angle, float len){
    float x_com, y_com;

    x_com = len*cos(angle);
    y_com = len*sin(angle);

    for(int i = 0; i < wallFrags.size(); i++){

        //Generate wall
        if(wallFrags[i].contains(robot.x + x_com, robot.y + y_com) && (wallFrags[i].getType() != Wall::typeWall)){ //Does a square exist on the recorded coords?

            //if a wall has been recorded, its type should be changed
            wallFrags[i].setType(Wall::typeWall);
            wallFrags[squareOccupy(robot.x + x_com, robot.y + y_com)].setSize(wallSize);

            break;
        }
        else if((i == (wallFrags.size() - 1)) && (spaceFree(robot.x + x_com, robot.y + y_com))){
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

void GUI::paintMap(){
    //Draw the rectangles the correspoding color
    for(int i = 0; i < wallFrags.size(); i++){
        XFillRectangle(display, window, (wallFrags[i].getType() == Wall::typeWall ? gcWall : gcEmpty), wallFrags[i].x - wallFrags[i].size.width/2, wallFrags[i].y - wallFrags[i].size.height/2, wallFrags[i].size.width, wallFrags[i].size.height);
    }
}

void GUI::paintRobot(){


    for(int i = 0; i < 4; i++){
        if(i < 3){
            XDrawLine(display, window, gcRobot, robot.pointsXAxis[i].x, robot.pointsXAxis[i].y, robot.pointsXAxis[i+1].x, robot.pointsXAxis[i+1].y);
        }
        else{
            XDrawLine(display, window, gcRobot, robot.pointsXAxis[i].x, robot.pointsXAxis[i].y, robot.pointsXAxis[0].x, robot.pointsXAxis[0].y);
        }
    }

    robot.rotatePoint(0.01); //Angle in radians

    XPutBackEvent(display, &event);
}

void GUI::update(bool& update){
    int update_counter = 0;
    QSqlQuery query;
    std::vector<double> angle;
    std::vector<double> distance;

    while(1){
        update_counter++;
        angle.clear();
        distance.clear();
        // This should only be true if the size of the query is above 0
        if (query.exec("SELECT status FROM lidar_data WHERE id = 1") && query.next()) {
            int status = query.value(0).toInt();
            if (status == 1) {
                if(update){
                    XSync(display, False);

        update_counter++;
        lidarReading(rand()%360, 200); //Make a timer function

        XSendEvent(display, window, False, ExposureMask, &event);

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

            /*if(update_counter >= 100){
                XClearWindow(display, window);
                update_counter = 0;
            }*/

            update = false;
        }

    }
}

void GUI::show(){
    XMapWindow(display, window);
    XNextEvent(display, &event);
}
