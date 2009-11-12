/*
 * Header File for the qtOpenGL Widget
 */

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "p3d_matrix.h"
#include "p3d_type.h"

#include "p3d_sys.h"

class Move3D2OpenGl;

class GLWidget: public QGLWidget
{
Q_OBJECT

public:
	GLWidget(QWidget *parent = 0);
	~GLWidget();

//	QSize minimumSizeHint() const;
//	QSize sizeHint() const;

	void setWinSize(double size);

	void resetImageVector();

public slots:
	void saveView();

	void addCurrentImage();
	void saveImagesToDisk();

	void setXRotation(int angle);
	void setYRotation(int angle);
	void setZRotation(int angle);
	void setZoomValue(int value);

	void paintNewGL();

	signals:
	void xRotationChanged(int angle);
	void yRotationChanged(int angle);
	void zRotationChanged(int angle);
	void zoomChanged(int value);

protected:
	void initializeGL();
	void paintGL();
	void resizeGL(int width, int height);
	void computeNewVectors(p3d_vector4& Xc,p3d_vector4& Xw,p3d_vector4& up);
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void keyPressEvent(QKeyEvent *e);
	void keyReleaseEvent(QKeyEvent *e);

private:
	void normalizeAngle(int *angle);

	GLdouble   x,y,z,el,az,zo;

	double size;

	p3d_vector4  up;

	int xRot;
	int yRot;
	int zRot;
	int zoomVal;

	QPoint lastPos;
	QColor trolltechGreen;
	QColor trolltechPurple;

	bool _light;
	bool _watingMouseRelease;

//	std::vector<QPixmap*> _pictures;
//	QVector<QPixmap*> _pictures;
};

extern int mouse_mode;

#endif
