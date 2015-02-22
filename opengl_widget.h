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
#include <vrpn_Button.h>
#include <vrpn_Analog.h>
#include <vrpn_Tracker.h>

class OpenGL_Widget : public QGLWidget
{
    Q_OBJECT

public:
    OpenGL_Widget(QWidget *parent = 0);
    ~OpenGL_Widget();

public slots:
	// Listen to a VRPN device with the specified name and
    // use the specified input to flash the background.
	void useVRPNButton(QString name, int which);
    void useVRPNAnalog(QString name, int which, double threshold);
    void useVRPNTracker(QString name, int sensor, double transThresh, double rotThresh);

    // Sets the number of quads that are drawn during a frame render.
    void setNumQuads(int num);

    // Called from within the class when nothing else is happening.
    void idle(void);

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

    int     d_num_quads;        //< How many quadrilateras to draw in the scene
    GLfloat *d_vertex_array;    //< Vertex array to use to draw the quads
    void updateVertexArray(void); //< Reallocate the vertex array.

    QTimer  d_timer;            //< Timer to use to cause oscillation
    bool    d_oscillate;        //< Should we be oscillating the cursor/display?
    double  d_oscillate_freq;   //< Cycles/second to oscillate cursor/display
    double  d_oscillate_phase;  //< Phase (in degrees) of the oscillation
    double  d_oscillate_mag;    //< Magnitude in pixels to oscillate
    QTime   d_osc_time;         //< Keeps track of how long we've been oscillating
    double  d_last_frame_time;  //< Last time we started rendering
    unsigned d_frame_count;     //< So we average frame rates
    std::string d_frame_msg;    //< Message use to store the frame rate.

    // Objects needed to be able to connect to a VRPN device.
    vrpn_Button_Remote *d_button;   //< If we are using a VRPN button, non-null
    int d_button_to_use;
    vrpn_Analog_Remote *d_analog;   //< If we are using a VRPN analog, non-null
    int d_analog_to_use;
    vrpn_float64 d_last_analog_value; //< Last analog value we read
    vrpn_float64 d_analog_threshold;  //< Threshold value triggering a change
    struct timeval d_analog_last_trigger;   //< When did we last trigger?
    vrpn_Tracker_Remote *d_tracker; //< If we are using a VRPN tracker, non-null
    double d_tracker_trans_thresh;  //< Threshold of translation to trigger
    double d_tracker_rot_thresh;    //< Threshold of rotation to trigger.
    vrpn_float64 d_tracker_last_pos[3]; //< Last location of the tracker
    vrpn_float64 d_tracker_last_quat[4]; //< Last orientation of the tracker
    struct timeval d_tracker_last_trigger;   //< When did we last trigger?

    // Callback handlers for the VRPN devices.
    static void VRPN_API handleButton(void *userdata, vrpn_BUTTONCB info);
    static void VRPN_API handleAnalog (void *userdata, vrpn_ANALOGCB info);
    static void VRPN_API handleTracker(void *userdata, vrpn_TRACKERCB info);
};

