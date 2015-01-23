#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Parse the command-line arguments
    bool do_fullscreen = false;
    std::string full = "-fullscreen";
    if ((argc == 2) && (full == argv[1])) {
        do_fullscreen = true;
    }

    MainWindow w(do_fullscreen);
    w.show();

    return a.exec();
}
