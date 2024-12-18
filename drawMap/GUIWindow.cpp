#include "GUIWindow.h"
#include <iostream>
#include <ostream>

GUI::GUI(){
    // Initialize the database connection
    db = QSqlDatabase::addDatabase("QMYSQL");
    db.setHostName("localhost");  // Set the hostname
    db.setDatabaseName("lidar_db");  // Set your database name
    db.setUserName("aksel");  // Set MySQL username
    db.setPassword("hua28rdc");  // Set MySQL password

    if (!db.open()) {
        std::cerr << "Database error: " << db.lastError().text().toStdString() << std::endl;
    }
    QSqlQuery query;
    query.prepare("UPDATE lidar_data SET is_read = FALSE WHERE is_read = TRUE");
    if (!query.exec()) {
        std::cerr << "Database error: " << query.lastError().text().toStdString() << std::endl;
    }

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

    //Create graphics contexts
    gcWall = XCreateGC(display, window, 0, NULL);
    gcEmpty = XCreateGC(display, window, 0, NULL);
    gcRobot = XCreateGC(display, window, 0, NULL);
    gcRed = XCreateGC(display, window, 0, NULL);
    gcGreen = XCreateGC(display, window, 0, NULL);

    //Color
    Colormap colorMap = DefaultColormap(display, screen);
    XColor color_Wall, color_Empty, color_Red, color_Green;

    XParseColor(display, colorMap, "snow4", &color_Empty);
    XAllocColor(display, colorMap, &color_Empty);

    XParseColor(display, colorMap, "firebrick3", &color_Red);
    XAllocColor(display, colorMap, &color_Red);

    XParseColor(display, colorMap, "springgreen4", &color_Green);
    XAllocColor(display, colorMap, &color_Green);

    XSetForeground(display, gcWall, BlackPixel(display, screen)); //Wall graphics
    XSetForeground(display, gcRed, color_Red.pixel); //Orient X
    XSetForeground(display, gcGreen, color_Green.pixel); //Orient Y

    //Robot initialization
    robot = Robot(Point(XDisplayWidth(display, screen)/2, XDisplayHeight(display, screen)/2), robotSize);

    //Create single wall to be able to iterate through the vector.
    wallFrags.push_back(Wall(Point(0,0), Size(round(10 * (scale_factor / 100)), round(15 * (scale_factor / 100)))));
}

GUI::~GUI(){
    XFreeGC(display, gcWall);
    XFreeGC(display, gcEmpty);
    XFreeGC(display, gcRed);
    XFreeGC(display, gcGreen);
    XCloseDisplay(display);
    db.close();
}

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
}

void GUI::movementRobot(float robot_x, float robot_y){
    // Update robot position to global coordinates
    robot.x = robot.start_x + robot_x * scale_factor;
    robot.y = robot.start_y + robot_y * scale_factor;
}

void GUI::rescale() {
    //if the robot comes near the edge, move it, and the walls.
    if(robot.x < 200){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].x = wallFrags[i].x + 200;
        }

        robot.start_x = robot.start_x + 200;
    }
    else if(robot.x > (XDisplayWidth(display, screen) - 200)){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].x = wallFrags[i].x - 200;
        }

        robot.start_x = robot.start_x - 200;
    }
    else if(robot.y < 200){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].y = wallFrags[i].y + 200;
        }

        robot.start_y = robot.start_y + 200;
    }
    else if(robot.y > (XDisplayHeight(display, screen) - 200)){
        for(int i = 0; i < wallFrags.size(); i++){
            wallFrags[i].y = wallFrags[i].y - 200;
        }

        robot.start_y = robot.start_y - 200;
    }
}

void GUI::paintMap(){
    //Draws the wallFrags
    for(int i = 0; i < wallFrags.size(); i++){
        XFillRectangle(display, window, gcWall, wallFrags[i].x - wallFrags[i].size.width/2, wallFrags[i].y - wallFrags[i].size.height/2, wallFrags[i].size.width, wallFrags[i].size.height);
    }
}

void GUI::findPoints(std::vector<Point>& points, Point p1, Point p2, bool TF){
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

    std::vector<Point> pointsStart; //Points between two vertices
    std::vector<Point> pointsEnd; //Points between the other two vertices

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

void GUI::update(bool& update){


    while(1){
        // Get data from the database, run the lidarReading function and update the is_read value in the database
        QSqlQuery query;
        int id_first = 0;
        int id_last = 0;
        query.prepare("SELECT * FROM lidar_data");
        if (!query.exec()) {
            std::cerr << "Database error: " << query.lastError().text().toStdString() << std::endl;
        } else {
            if(query.first()){
                id_first = query.value(0).toInt();
                double angle = query.value(1).toDouble();
                double distance = query.value(2).toDouble();
                double robot_x = query.value(3).toDouble();
                double robot_y = query.value(4).toDouble() - 0.7;
                double robot_yaw = query.value(5).toDouble();
                movementRobot(robot_x, robot_y);
                lidarReading(angle, distance);
                paintMap();
            }
            while (query.next()) {
                id_last = query.value(0).toInt();
                double angle = query.value(1).toDouble();
                double distance = query.value(2).toDouble();
                double robot_x = query.value(3).toDouble();
                double robot_y = query.value(4).toDouble() - 0.7;
                double robot_yaw = query.value(5).toDouble();
                movementRobot(robot_x, robot_y);
                lidarReading(angle, distance);
                paintMap();
            }
            query.prepare("UPDATE lidar_data SET is_read = TRUE WHERE id BETWEEN :id_first AND :id_last");
            query.bindValue(":id_first", id_first);
            query.bindValue(":id_last", id_last);
            if (!query.exec()) {
                std::cerr << "Database error: " << query.lastError().text().toStdString() << std::endl;
            }
        }

        paintMap();

        if(XPending(display) > 0){ //Is there an Event waiting to be drawn?
            XNextEvent(display, &event); //Select the next event to be drawn.
            if(event.type == Expose){
                update = true; //Allow the screen to update.
            }
        }

        if(update){ //Update the screen
            XSync(display, False); //Sync the screen with the data in the system
            XClearWindow(display, window); //Clear the window, needs to be after XSync, for some reason.
            update = false; //Stop the screen from continously updating
        }
    }
}

void GUI::show(){
    XMapWindow(display, window);
    XNextEvent(display, &event);
}
