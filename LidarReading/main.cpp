#include "windowClass.h"

#include <iostream>
#include <conio.h>
#include <math.h>
#include <QApplication>
#include <QSurfaceFormat>

using namespace std;

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QSurfaceFormat fmt;
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    Window GUI;

    GUI.show();

    return app.exec();
}
