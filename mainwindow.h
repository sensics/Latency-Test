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
        , QString buttonName = "", int whichButton = 0
        , QString analogName = "", int whichAnalog = 0, double anaThresh = 0
        , QString trackerName = "", int whichSensor = 0, double transThresh = 0, double rotThresh = 0
        , QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
