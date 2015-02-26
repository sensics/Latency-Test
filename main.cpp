#include <QApplication>
#include "mainwindow.h"
#include <iostream>

static void usage(std::string name)
{
    std::cerr << "Usage: " << name << " [-fullscreen]"
        << " [-button vrpnDeviceName whichButton]"
        << " [-analog vrpnDeviceName whichAnalog threshold]"
        << " [-tracker vrpnDeviceName whichSensor translationThreshold rotationThreshold]"
        << " [-trackerRotate vrpnDeviceName whichSensor whichAxis[0,1,2]]"
        << " [-numquads num]"
        << std::endl;
    exit(-1);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Parse the command-line arguments
    int real_params = 0;
    bool doFullscreen = false;
    std::string buttonName, analogName, trackerName;
    int whichButton, whichAnalog, whichSensor;
    double analogThreshold, transThreshold, rotThreshold;
    bool doTrackerRotation = false;
    int rotateAxis = -1;
    int numQuads = -1; //-1 is invalid, so uses the default
    for (int i = 1; i < argc; i++) {
        if (std::string("-fullscreen") == argv[i]) {
            doFullscreen = true;
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
        else if (std::string("-trackerRotate") == argv[i]) {
            if (++i >= argc) { usage(argv[0]); }
            trackerName = argv[i];
            if (++i >= argc) { usage(argv[0]); }
            whichSensor = atoi(argv[i]);
            if (++i >= argc) { usage(argv[0]); }
            doTrackerRotation = true;
            rotateAxis = atoi(argv[i]);
        }
        else if (std::string("-numquads") == argv[i]) {
            if (++i >= argc) { usage(argv[0]); }
            numQuads = atoi(argv[i]);
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

    MainWindow w(
        doFullscreen
        , numQuads
        , buttonName.c_str(), whichButton
        , analogName.c_str(), whichAnalog, analogThreshold
        , trackerName.c_str(), whichSensor, transThreshold, rotThreshold
        , doTrackerRotation, rotateAxis);
    w.show();

    return a.exec();
}
