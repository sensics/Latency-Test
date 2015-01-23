#-------------------------------------------------
#
# Project created by QtCreator 2014-12-22T17:42:44
#
#-------------------------------------------------

# Modified to work with Qt5.

QT       += core gui opengl widgets

TARGET = calibration
TEMPLATE = app

# Avoid some warnings on Windows
DEFINES += _CRT_SECURE_NO_WARNINGS=1

SOURCES += main.cpp\
        mainwindow.cpp \
    opengl_widget.cpp 

HEADERS  += mainwindow.h \
    opengl_widget.h 

FORMS    += mainwindow.ui
