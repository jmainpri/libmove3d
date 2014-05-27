/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
/*
 * Header File for the qtOpenGL Widget
 */

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "Graphic-pkg.h"

#undef Status
#undef Black
#undef Bool
#undef CursorShape
#undef None
#undef KeyPress
#undef KeyRelease
#undef FocusIn
#undef FocusOut
#undef FontChange
#undef Unsorted

#include <QtOpenGL/QGLWidget>
#include <QtCore/QObject>

#include "../../graphic/proto/g3d_newWindow.hpp"

class Move3D2OpenGl;

void qt_get_win_mouse(int* i, int* j);
void qt_ui_calc_param(g3d_cam_param& p);

/**
 * @ingroup qtWindow
 * @brief Open GL viewer implemetation in Qt
 */
class GLWidget: public QGLWidget
{
	Q_OBJECT
	
public:
	GLWidget(QWidget *parent = 0);
	~GLWidget();
	
	//void setMainWindow(MainWindow* w) { m_mainWindow = w; }
	void setWinSize(double size);
        void setWinSize(int x, int y);
	void resetImageVector();
	void setThreadWorking(bool isWorking);
	void newG3dWindow();
	void initG3DFunctions();
        void setAutoBufferSwapping(bool swapping);
	
	public slots:
	void saveView();
	void reinitGraphics();
	
	void addCurrentImage();
	void saveImagesToDisk();
	
signals:
	void xRotationChanged(int angle);
	void yRotationChanged(int angle);
	void zRotationChanged(int angle);
	void zoomChanged(int value);
	
protected:
	// OpenGL functions
	void initializeGL();
	void paintGL(); // This is called when changing environments.
	void resizeGL(int width, int height);
	void computeNewVectors(p3d_vector4& Xc,p3d_vector4& Xw,p3d_vector4& up);
	
	// Mouse events
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void keyPressEvent(QKeyEvent *e);
	void keyReleaseEvent(QKeyEvent *e);
	void mouseDoubleClickEvent(QMouseEvent *event);
	
private:
	// Pointer that allows resizing
	//MainWindow*	m_mainWindow;
	
	// OpenGl variables
	GLdouble   x,y,z,el,az,zo;
	
	// size of the OpenGl scene
	int _w,_h;
	double _size;
	
	p3d_vector4  up;
	
	QPoint lastPos;
	
	// Colors for background
	QColor trolltechGreen;
	QColor trolltechPurple;
	QColor trolltechGrey;
	QColor trolltechBlack;
	QColor trolltechWhite;
	
	bool _light;
	bool _watingMouseRelease;
	
	// Do not draw when this 
	// variable is true
	bool _isThreadWorking;
	
	// Vector of recorded images
	QVector<QImage*> _pictures;
	
	// Counts the number of draw
	int paintNum;
	
#ifndef WITH_XFORMS
	qtG3DWindow* mG3DOld;
#endif
};

extern int mouse_mode;

#endif
