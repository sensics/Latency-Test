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
// All rights reserved.
//
// (Final version intended to be licensed under
// the Apache License, Version 2.0)

#include "opengl_widget.h"
#include "mainwindow.h"

#include <QtGui>
#include <QtOpenGL>
#include <QColor>
#include <QFileDialog>
#include <QCursor>
#include <QMainWindow>
#include <sstream>
#include <math.h>
#include <stdio.h>

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

//----------------------------------------------------------------------
// Helper functions

OpenGL_Widget::OpenGL_Widget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    d_last_frame_time = 0;
    d_frame_count = 0;

    // Initial target radius is 5 pixels.
    d_r = 12;

    // Initial clear color is black;
    d_clearColor = Qt::black;

    // Start out oscillating the cursor and display
    d_oscillate = true;
    d_oscillate_freq = 2;
    d_oscillate_phase = 0;
    d_oscillate_mag = 60;

    // Call the timer that will cause OpenGL to draw
    // faster than the maximum display update rate.
    connect(&d_timer,SIGNAL(timeout()),this,SLOT(oscillate()));
    d_osc_time.start();
    d_timer.start();

    // Set a cursor that is symmetric around its center.
    this->setCursor(Qt::CrossCursor);
}

OpenGL_Widget::~OpenGL_Widget()
{
}

void OpenGL_Widget::oscillate(void)
{
    updateGL();
}

void OpenGL_Widget::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glDisable(GL_MULTISAMPLE);

    // Makes the colors for the primitives be what we want.
    glDisable(GL_BLEND);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
}

void OpenGL_Widget::paintGL()
{
    if (d_oscillate) {
        // Figure out the new location in the oscillation.  Move both
        // the cursor and the displayed box to their respective locations.
        // Call code to move the cursor here.  Put this code immediately
        // before the 3D rendering loop so there is not additional latency
        // due to intervening operations.
        double ms = d_osc_time.elapsed();
        double sec = ms / 1000;
        int x = d_width / 2 + d_oscillate_mag * cos(sec * (2* M_PI * d_oscillate_freq));
        int y = d_height / 2 + d_oscillate_mag * sin(sec * (2* M_PI * d_oscillate_freq));
        double phase_cycles = d_oscillate_phase / 360.0;
        int box_x = d_width / 2 + d_oscillate_mag *
                cos(2* M_PI * (sec * d_oscillate_freq + phase_cycles));
        int box_y = d_height / 2 + d_oscillate_mag *
                sin(2* M_PI * (sec * d_oscillate_freq + phase_cycles));
        QCursor::setPos(QWidget::mapToGlobal(QPoint(x,y)));
        d_x = box_x;
        d_y = d_height - box_y;    // Y inverted for OpenGL and mouse
    }

    qglClearColor(d_clearColor);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -10.0);

    // Draw a square box at the specified coordinates.
    glColor3f(1,0,0);
    glBegin(GL_QUADS);
        glVertex2d(d_x - d_r, d_y - d_r);
        glVertex2d(d_x + d_r, d_y - d_r);
        glVertex2d(d_x + d_r, d_y + d_r);
        glVertex2d(d_x - d_r, d_y + d_r);
    glEnd();

    // Draw instructions on the screen.  This will provide additional rendering load,
    // but it did not change the results in our early testing.
    glColor3f(1,1,1);
    renderText(30, 30, "OSVR 2D vs. 3D rendering latency test program version 01.02.01 (run with -fullscreen to remove borders)");
    renderText(50, 50, "Press + to increase oscillation");
    renderText(50, 70, "Press - to decrease oscillation");
    renderText(50, 90, "Until you find the slowest oscillation where the cursor and square are in phase");

    renderText(50,110, "Press o to toggle oscillation");
    renderText(50,130, "Press ESC/q to quit");
    renderText(50,150, "Left mouse button drags square when not oscillating");
    renderText(50,170, "Middle mouse button teleports mouse when not oscillating");
    renderText(50,190, "Right mouse button toggles background brightness");

    double latency = (d_oscillate_phase/360) / (d_oscillate_freq);
    double latency_ms = latency * 1000;
    std::stringstream msg;
    msg << "Latency = " << latency_ms << "ms, Phase = " << d_oscillate_phase << " degrees";
    renderText(50,220, msg.str().c_str());

    if (++d_frame_count == 30) {
        double now = d_osc_time.elapsed() / 1000.0;
        double duration = now - d_last_frame_time;
        msg.str("");
        msg << "Frame rate = " << d_frame_count/duration << " Hz";
        d_frame_msg = msg.str();
        d_last_frame_time = now;
        d_frame_count = 0;
    }
    renderText(50,240, d_frame_msg.c_str());
}

void OpenGL_Widget::resizeGL(int width, int height)
{
    d_width = width;
    d_height = height;
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // Make the window one unit high (-0.5 to 0.5) and have an aspect ratio that matches
    // the aspect ratio of the window.  We also make the left side of the window be at
    // the origin.
    float aspect;
    if ((height <= 0) || (width < 0)) {
        aspect = 1.0;
    } else {
        aspect = static_cast<float>(width)/height;
    }
    glOrtho(0, d_width-1, 0, d_height-1, 5.0, 15.0);
    glMatrixMode(GL_MODELVIEW);
    d_x = d_width / 2;
    d_y = d_height / 2;    
}


void OpenGL_Widget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Escape:
    case Qt::Key_Q:
        //d_timer->stop();
        QApplication::quit();
        break;
    case Qt::Key_Plus:
        d_oscillate_phase += 2.5;
        break;
    case Qt::Key_Minus:
        d_oscillate_phase -= 2.5;
        break;
    case Qt::Key_O:
        d_oscillate = !d_oscillate;
        break;
    case Qt::Key_Left:
        break;
    case Qt::Key_Right:
        break;
    case Qt::Key_Down:
        break;
    case Qt::Key_Up:
        break;
    case Qt::Key_R:
        break;
    }

    updateGL();
}

void OpenGL_Widget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        // Invert the mouse y position because the coordinates are
        // upside down in OpenGL compared to the mouse.
        d_x = event->x();
        d_y = d_height - event->y();
    }
    if (event->button() == Qt::RightButton) {
        d_clearColor = Qt::white;
    }
    if (event->button() == Qt::MiddleButton) {
        QPoint newPos = event->pos();
        newPos.setX(newPos.x() - 50);
        newPos.setY(newPos.y() - 50);
        QCursor::setPos(QWidget::mapToGlobal(newPos));
    }
    updateGL();
}

void OpenGL_Widget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton) {
        d_clearColor = Qt::black;
    }
    if (event->button() == Qt::MiddleButton) {
        QPoint newPos = event->pos();
        newPos.setX(newPos.x() + 50);
        newPos.setY(newPos.y() + 50);
        QCursor::setPos(QWidget::mapToGlobal(newPos));
    }
    updateGL();
}

void OpenGL_Widget::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
        // Invert the mouse y position because the coordinates are
        // upside down in OpenGL compared to the mouse.
        d_x = event->x();
        d_y = d_height - event->y();
    } else if (event->buttons() & Qt::RightButton) {
    }
    updateGL();
}
