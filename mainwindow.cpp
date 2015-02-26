#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDesktopWidget>

MainWindow::MainWindow(bool fullscreen
    , int numQuads
    , QString buttonName, int whichButton
    , QString analogName, int whichAnalog, double anaThresh
    , QString trackerName, int whichSensor, double transThresh, double rotThresh
    , bool doTrackerRotation, int rotateAxis
    , QWidget *parent)
  : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Make the central widget have no margins, so we fill the whole region.
    QMainWindow::centralWidget()->layout()->setContentsMargins(0, 0, 0, 0);

    // Render in full screen if we've been asked to.
    if (fullscreen) {
        // Find out where the last screen lives.
        int last_screen = QApplication::desktop()->numScreens() - 1;
        QRect screenres = QApplication::desktop()->screenGeometry(last_screen);

        // Display full resolution on the last screen.
        // On Qt4, the showFullScreen() should come after the move/resize.
        // On Qt5, it comes before.
        this->showFullScreen();
        this->move(QPoint(screenres.x(), screenres.y()));
        this->resize(screenres.width(), screenres.height());
    }

    // Set up the VRPN devices if we've been asked to.
    if (buttonName.length() > 0) {
        ui->widget->useVRPNButton(buttonName, whichButton);
    }
    if (analogName.length() > 0) {
        ui->widget->useVRPNAnalog(analogName, whichAnalog, anaThresh);
    }
    if (trackerName.length() > 0) {
        if (doTrackerRotation) {
            ui->widget->useVRPNTrackerRotate(trackerName, whichSensor, rotateAxis);
        }
        else {
            ui->widget->useVRPNTracker(trackerName, whichSensor, transThresh, rotThresh);
        }
    }
    if (numQuads > 0) {
        ui->widget->setNumQuads(numQuads);
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
