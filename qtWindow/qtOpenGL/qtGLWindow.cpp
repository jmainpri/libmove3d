/*
 * Source File for the
 * main openGL window
 */

#include "glwidget.hpp"
#include "qtGLWindow.hpp"

#include "Move3d-pkg.h"
#include "P3d-pkg.h"

qtGLWindow::qtGLWindow()
{
	glWidget = new GLWidget;
	win = G3D_WIN;
	
#ifndef WITH_XFORMS	
	g3d_set_win_floor_color(g3d_get_cur_win(), 0.5, 1.0, 1.0);
	//  g3d_set_win_bgcolor(g3d_get_cur_win(), 0.5, 0.6, 1.0);
	g3d_set_win_wall_color(g3d_get_cur_win(), 0.4, 0.45, 0.5);
	g3d_set_win_bgcolor(g3d_get_cur_win(), XYZ_ENV->background_color[0], XYZ_ENV->background_color[1], XYZ_ENV->background_color[2]);
#endif
	
	glWidget->setWinSize(win->size);

	/*xSlider = createSlider();
	ySlider = createSlider();
	zSlider = createSlider();
	zoom = createSlider();*/

	createCheckBoxes();

	opfloor->setChecked(true);
	optiles->setChecked(true);
	walls->setChecked(true);

	/*connect(xSlider, SIGNAL(valueChanged(int)), glWidget,
			SLOT(setXRotation(int)));
	connect(glWidget, SIGNAL(xRotationChanged(int)), xSlider,
			SLOT(setValue(int)));
	connect(ySlider, SIGNAL(valueChanged(int)), glWidget,
			SLOT(setYRotation(int)));
	connect(glWidget, SIGNAL(yRotationChanged(int)), ySlider,
			SLOT(setValue(int)));
	connect(zSlider, SIGNAL(valueChanged(int)), glWidget,
			SLOT(setZRotation(int)));
	connect(glWidget, SIGNAL(zRotationChanged(int)), zSlider,
			SLOT(setValue(int)));
	connect(zoom, SIGNAL(valueChanged(int)), glWidget,
				SLOT(setZoomValue(int)));
	connect(glWidget, SIGNAL(zoomChanged(int)), zoom,
				SLOT(setValue(int)));*/

	QHBoxLayout *mainLayout = new QHBoxLayout;
	mainLayout->addWidget(glWidget);

	/*mainLayout->addWidget(xSlider);
	mainLayout->addWidget(ySlider);
	mainLayout->addWidget(zSlider);*/

	/*mainLayout->addWidget(zoom);*/

	QPushButton* saveView = new QPushButton("Save View");
	connect(saveView,SIGNAL(clicked()),glWidget, SLOT(saveView()));

	QBoxLayout* box = new QBoxLayout(QBoxLayout::TopToBottom);
	box->addWidget(vGhost);
	box->addWidget(vBb);
	box->addWidget(opfloor);
	box->addWidget(optiles);
	box->addWidget(walls);
	box->addWidget(shadows);

	box->addWidget(saveView);

	mainLayout->addLayout(box,0);

	setLayout(mainLayout);

	/*xSlider->setValue(15 * 160);
	ySlider->setValue(345 * 160);
	zSlider->setValue(0 * 160);
	zSlider->setValue(10);*/
	setWindowTitle(tr("Move3D"));
//	setSizeIncrement(800,600);
}

QSlider *qtGLWindow::createSlider()
{
	QSlider *slider = new QSlider(Qt::Vertical);
	slider->setRange(0, 360 * 160);
	slider->setSingleStep(160);
	slider->setPageStep(15 * 160);
	slider->setTickInterval(15 * 160);
	slider->setTickPosition(QSlider::TicksRight);
	return slider;
}

void qtGLWindow::createCheckBoxes()
{
	QString str;

	str = "Ghost";
	vGhost = new QCheckBox(str);
	connect(vGhost, SIGNAL(toggled(bool)), this , SLOT(setBoolGhost(bool)), Qt::DirectConnection);
	connect(vGhost, SIGNAL(toggled(bool)), glWidget , SLOT(updateGL()));

	str = "Bb";
	vBb = new QCheckBox(str);
	connect(vBb, SIGNAL(toggled(bool)), this , SLOT(setBoolBb(bool)), Qt::DirectConnection);
	connect(vBb, SIGNAL(toggled(bool)), glWidget , SLOT(updateGL()));
//	vBb->setChecked(ENV.getBool(p));

	str = "Floor";
	opfloor = new QCheckBox(str);
	connect(opfloor, SIGNAL(toggled(bool)), this , SLOT(setBoolFloor(bool)), Qt::DirectConnection);
	connect(opfloor, SIGNAL(toggled(bool)), glWidget , SLOT(updateGL()));

	str = "Tiles";
	optiles = new QCheckBox(str);
	connect(optiles, SIGNAL(toggled(bool)), this , SLOT(setBoolTiles(bool)), Qt::DirectConnection);
	connect(optiles, SIGNAL(toggled(bool)), glWidget , SLOT(updateGL()));

	str = "Walls";
	walls = new QCheckBox(str);
	connect(walls, SIGNAL(toggled(bool)), this , SLOT(setBoolWalls(bool)), Qt::DirectConnection);
	connect(walls, SIGNAL(toggled(bool)), glWidget , SLOT(updateGL()));

	str = "Shadows";
	shadows = new QCheckBox(str);
	connect(shadows, SIGNAL(toggled(bool)), this , SLOT(setBoolShadows(bool)), Qt::DirectConnection);
	connect(shadows, SIGNAL(toggled(bool)), glWidget , SLOT(updateGL()));
}

void qtGLWindow::setBoolGhost(bool value)
{
	win->GHOST = value;
}

void qtGLWindow::setBoolBb(bool value)
{
	win->BB = value;
}


void qtGLWindow::setBoolFloor(bool value)
{
	win->displayFloor = value;
}


void qtGLWindow::setBoolTiles(bool value)
{
	win->displayTiles = value;
}


void qtGLWindow::setBoolWalls(bool value)
{
	win->displayWalls = value;
}

void qtGLWindow::setBoolShadows(bool value)
{
	win->displayShadows = value;
}
