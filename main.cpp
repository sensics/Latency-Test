#include <QApplication>
#include "mainwindow.h"
#include <iostream>

static void usage(std::string name)
{
    std::cerr << "Usage: " << name << " [-fullscreen]" <<
        " [-button vrpnDeviceName whichButton]" <<
        " [-analog vrpnDeviceName whichAnalog threshold]" <<
        " [-tracker vrpnDeviceName whichSensor translationThreshold rotationThreshold]"
        << std::endl;
    exit(-1);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Parse the command-line arguments
    int real_params = 0;
    bool do_fullscreen = false;
    std::string buttonName, analogName, trackerName;
    int whichButton, whichAnalog, whichSensor;
    double analogThreshold, transThreshold, rotThreshold;
    for (int i = 1; i < argc; i++) {
        if (std::string("-fullscreen") == argv[i]) {
            do_fullscreen = true;
        }
        else if (std::string("-button") == argv[i]) {
            if (++i >= argc) { usage(argv[0]); }
            buttonName = argv[i];
            if (++i >= argc) { usage(argv[0]); }
            whichButton = atoi(argv[i]);
        }
        else if (std::string("-analog") == argv[i]) {
            if (++i >= argc) { usage(argv[0]); }
            analogName = argv[i];
            if (++i >= argc) { usage(argv[0]); }
            whichAnalog = atoi(argv[i]);
            if (++i >= argc) { usage(argv[0]); }
            analogThreshold = atof(argv[i]);
        }
        else if (std::string("-tracker") == argv[i]) {
            if (++i >= argc) { usage(argv[0]); }
            trackerName = argv[i];
            if (++i >= argc) { usage(argv[0]); }
            whichSensor = atoi(argv[i]);
            if (++i >= argc) { usage(argv[0]); }
            transThreshold = atof(argv[i]);
            if (++i >= argc) { usage(argv[0]); }
            rotThreshold = atof(argv[i]);
        }
        else if (argv[i][0] == '-') {
            usage(argv[0]);
        }
        else switch (++real_params) {
        case 1:
        case 2:
        default:
            usage(argv[0]);
        }
    }
    if (real_params != 0) {
        usage(argv[0]);
    }

    MainWindow w(do_fullscreen
        , buttonName.c_str(), whichButton
        , analogName.c_str(), whichAnalog, analogThreshold
        , trackerName.c_str(), whichSensor, transThreshold, rotThreshold);
    w.show();

    return a.exec();
}
