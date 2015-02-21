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

#pragma once
#include "opengl_widget.h"
#include <QGLWidget>
#include <QTimer>
#include <QTime>

class OpenGL_Widget : public QGLWidget
{
    Q_OBJECT

public:
    OpenGL_Widget(QWidget *parent = 0);
    ~OpenGL_Widget();

public slots:
    // Called when nothing else is happening.
    void oscillate(void);

signals:

protected:
    //------------------------------------------------------
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);

    //------------------------------------------------------
    // Helper functions for the draw routines.

private:
    int d_width, d_height;  //< Size of the window we're rendering into
    int d_x, d_y;           //< Location to draw the target.
    float d_r;              //< Radius of the target

    QColor  d_clearColor;   //< Color to clear the background to

    QTimer  d_timer;            //< Timer to use to cause oscillation
    bool    d_oscillate;        //< Should we be oscillating the cursor/display?
    double  d_oscillate_freq;   //< Cycles/second to oscillate cursor/display
    double  d_oscillate_phase;  //< Phase (in degrees) of the oscillation
    double  d_oscillate_mag;    //< Magnitude in pixels to oscillate
    QTime   d_osc_time;         //< Keeps track of how long we've been oscillating
    double  d_last_frame_time;  //< Last time we started rendering
    unsigned d_frame_count;     //< So we average frame rates
    std::string d_frame_msg;    //< Message use to store the frame rate.
};
