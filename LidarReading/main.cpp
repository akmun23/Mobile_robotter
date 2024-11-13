#include "windowClass.h"
#include <math.h>
#include <QApplication>
#include <QSurfaceFormat>

using namespace std;

int main(int argc, char *argv[])
{
    // Setup of GUI
    QApplication app(argc, argv);
    QSurfaceFormat fmt;
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);
    Window GUI;
    GUI.show();
    app.exec();

    return 0;
}
