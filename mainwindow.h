/** @file
@brief Header

@date 2015

@author
Russell Taylor
<russ@reliasolve.com>
<http://sensics.com>
*/

// Copyright 2015 Sensics, Inc.
//
// (Licensed under the Apache License, Version 2.0)

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(bool fullscreen = true
        , int numQuads = -1  //< How many quads to draw during render (-1 means default of 1)
        , QString buttonName = "", int whichButton = 0
        , QString analogName = "", int whichAnalog = 0, double anaThresh = 0
        , QString trackerName = "", int whichSensor = 0, double transThresh = 0, double rotThresh = 0
        , bool doTrackerRotation = false, int rotateAxis = 0
        , QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
