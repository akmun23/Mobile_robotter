#include "GUIWindow.h"
#include <iostream>
#include <cmath>

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

    //Setting variables
    emptySize = Size(1,1);
    wallSize = Size(8,8);

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

    //Create single wall to be able to iterate through the vector.
    wallFrags.push_back(Wall(Point(0,0), emptySize));
}

GUI::~GUI(){
    XFreeGC(display, gcWall);
    XFreeGC(display, gcEmpty);
    XFreeGC(display, gcRed);
    XFreeGC(display, gcGreen);
    XCloseDisplay(display);
    db.close();
}

bool GUI::spaceFree(int x_, int y_){ //Returns true if no wallFrags was found to contain the given point
    for(int i = 0; i < wallFrags.size(); i++){
        if(wallFrags[i].contains(x_, y_)){
            return false;
        }
    }
    return true;
}

void GUI::lidarReading(double angle, double len, double robot_x, double robot_y, double robot_yaw){
    float x_com, y_com;

    //Calculate the x and y components
    x_com = (XDisplayWidth(display, screen)/2) + (len * cos(angle + robot_yaw) * scale_factor) + (robot_x * scale_factor);
    y_com = (XDisplayHeight(display, screen)/2) + (len * sin(angle + robot_yaw) * scale_factor) + (robot_y * scale_factor);

    std::cout << "x_com: " << x_com << " y_com: " << y_com << std::endl;

    if(spaceFree(x_com, y_com)){ //if there is no wall drawn at the detected point, create a new wall.
        wallFrags.push_back(Wall(Point(x_com, y_com), wallSize, Wall::typeWall));
    }
}

void GUI::paintMap(){
    //Draws the wallFrags
    for(int i = 0; i < wallFrags.size(); i++){
        XFillRectangle(display, window, gcWall, wallFrags[i].x - wallFrags[i].size.width/2, wallFrags[i].y - wallFrags[i].size.height/2, wallFrags[i].size.width, wallFrags[i].size.height);
    }
}

void GUI::update(bool& update){
    // Get data from the database and run the lidarReading function
    QSqlQuery query;
    query.prepare("SELECT * FROM lidar_data");
    if (!query.exec()) {
        std::cerr << "Database error: " << query.lastError().text().toStdString() << std::endl;
    } else {
        while (query.next()) {
            double angle = query.value(1).toDouble();
            double distance = query.value(2).toDouble();
            double robot_x = query.value(3).toDouble();
            double robot_y = query.value(4).toDouble();
            double robot_yaw = query.value(5).toDouble();

            lidarReading(angle, distance, robot_x, robot_y, robot_yaw);
            paintMap();
        }
        std::cout << "All data from the database has been read." << std::endl;
    }

    while(1){
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
