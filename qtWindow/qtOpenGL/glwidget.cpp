/*
 * Source File for the Open GL Window
 */

#include "glwidget.hpp"

//#include "Planner-pkg.h"
//#include "Move3d-pkg.h"
//#include "P3d-pkg.h"
//#include "Graphic-pkg.h"
//
//#include <iostream>

#include "../../planner_cxx/API/planningAPI.hpp"

using namespace std;

int mouse_mode = 0;

extern void* GroundCostObj;

GLWidget::GLWidget(QWidget *parent) :
	QGLWidget(parent)
{
	xRot = 0;
	yRot = 0;
	zRot = 0;

	x = 0;
	y = 0;
	z = 0;

	up[0] = 0;
	up[1] = 0;
	up[2] = 0;
	up[3] = 0;

	az = INIT_AZ;
	el = INIT_EL;
	double x1, x2, y1, y2, z1, z2;
	p3d_get_env_box(&x1, &x2, &y1, &y2, &z1, &z2);
	zo = 0.2 * MAX(MAX(x2-x1,y2-y1),z2-z1);

	trolltechGreen = QColor::fromCmykF(0.40, 0.0, 1.0, 0.0);
	trolltechPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);
        trolltechGrey = QColor::fromCmykF(0.5, 0.5, 0.5, 0.0);
        trolltechBlack = QColor::fromCmykF(1.0, 1.0, 1.0, 0.0);
        trolltechWhite =  QColor::fromCmykF(0.0, 0.0, 0.0, 0.0);

        _isThreadWorking = false;
        _light = false;

#ifndef WITH_XFORMS
        mG3DOld = new qtG3DWindow;
#endif

//	setFocusPolicy(Qt::StrongFocus);
}


GLWidget::~GLWidget()
{
	makeCurrent();
}

/*QSize GLWidget::minimumSizeHint() const
{
        return QSize(400, 300);
}

QSize GLWidget::sizeHint() const
{
        return QSize(1600, 1200);
}*/

void GLWidget::setWinSize(double size)
{
	this->size = size;
}

QVector<QImage*> _pictures;

void GLWidget::addCurrentImage()
{
	//	QPixmap* image = new QPixmap(renderPixmap());
	QImage* image = new QImage(grabFrameBuffer());
	_pictures.push_back(image);
}

void GLWidget::saveImagesToDisk()
{
	ostringstream oss(ostringstream::out);
	oss << "cd "<< getenv("HOME_MOVE3D") <<"/video;rm *.jpg";
	system(oss.str().c_str());

	for (int i = 0; i < _pictures.size(); i++)
	{
		oss.str("");
		oss << "video/Image_" << setfill('0') << setw(4) << i << ".jpg";
		cout << "Saving : " << oss.str() << endl;
		_pictures.at(i)->save(oss.str().c_str(), "JPG", 100);
	}
	resetImageVector();
	cout << "Images saved to video/" << endl;

	oss.str("");
	//change to video directory then compress jpg files to AVI video for more parameters and video format see man pages of mencoder
	oss << "cd "<<getenv("HOME_MOVE3D")<<"/video;mencoder mf://*.jpg -mf w=800:h=600:fps=25:type=jpeg -ovc lavc -lavcopts vcodec=mpeg4 -oac copy -o output.avi";
	system(oss.str().c_str());
}

void GLWidget::resetImageVector()
{
	for (int i = 0; i < _pictures.size(); i++)
	{
		delete _pictures.at(i);
	}
	_pictures.clear();
}

void GLWidget::saveView()
{
	QImage image = grabFrameBuffer();
	//	QPixmap image = renderPixmap(0,0,false);
	image.save("OpenGLView.jpg", "JPG", 100);
}

void GLWidget::setXRotation(int angle)
{
	normalizeAngle(&angle);
	if (angle != xRot)
	{
		xRot = angle;
		emit
		xRotationChanged(angle);

		if (pipe2openGl)
		{
			pipe2openGl->updatePipe();
		}
	}
}

void GLWidget::setYRotation(int angle)
{
	normalizeAngle(&angle);
	if (angle != yRot)
	{
		yRot = angle;
		emit
		yRotationChanged(angle);

		if (pipe2openGl)
		{
			pipe2openGl->updatePipe();
		}

	}
}

void GLWidget::setZRotation(int angle)
{
	normalizeAngle(&angle);
	if (angle != zRot)
	{
		zRot = angle;
		emit
		zRotationChanged(angle);

		if (pipe2openGl)
		{
			pipe2openGl->updatePipe();
		}
	}
}

void GLWidget::setZoomValue(int value)
{
	if (value != zoomVal)
	{
		//		zoomVal = value;
		//		emit
		//		zoomChanged(value);
		//
		this->zo = zoomVal;
		if (this->zo < .0)
			this->zo = .0;

		if (pipe2openGl)
		{
			pipe2openGl->updatePipe();
		}
	}
}

p3d_vector4 JimXc;
p3d_vector4 JimXw;
p3d_vector4 Jimup;

void GLWidget::initializeGL()
{

	//	object = makeObject();
	//	glShadeModel(GL_FLAT);
	//	glEnable(GL_DEPTH_TEST);
	//	glEnable(GL_CULL_FACE);
	//	glViewport(0, 0, (GLint) 800, (GLint) 600);
	//	glMatrixMode(GL_PROJECTION);
	//	glLoadIdentity();
	//	gluPerspective(40.0, (GLdouble) 800 / (GLdouble) 600, 1.0, 5000.0);

	glViewport(0, 0, (GLint) 800, (GLint) 600);
//	qglClearColor(QColor::fromCmykF(0.0, 0.0, 0.0, 0.0));

        if(!GroundCostObj)
        {
            qglClearColor(trolltechGrey);
        }
        else
        {
            qglClearColor(trolltechWhite);
            G3D_WIN->displayFloor = false;
        }

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(40.0, (GLdouble) 800 / (GLdouble) 600, size / 1000., size
			* 1000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//	  paintNewGL();
}

void GLWidget::setThreadWorking(bool isWorking)
{
     _isThreadWorking = isWorking;
}

int paintNum = 0;
bool QGroupBox = false;
GLuint listBoite;
void GLWidget::paintGL()
{
    if(ENV.getBool(Env::drawDisabled))
    {
        return;
    }
//        cout << "paintGL()" << endl;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();

	p3d_vector4 Xc, Xw;
	p3d_vector4 up;

	//	computeNewVectors(Xc,Xw,up);

	qt_calc_cam_param();

	Xc[0] = JimXc[0];
	Xc[1] = JimXc[1];
	Xc[2] = JimXc[2];

	Xw[0] = JimXw[0];
	Xw[1] = JimXw[1];
	Xw[2] = JimXw[2];

	up[0] = Jimup[0];
	up[1] = Jimup[1];
	up[2] = Jimup[2];

	gluLookAt(Xc[0], Xc[1], Xc[2], Xw[0], Xw[1], Xw[2], up[0], up[1], up[2]);

#ifdef WITH_XFORMS
	if ((lockDrawAllWin != 0) && (waitDrawAllWin != 0))
	{
		lockDrawAllWin->lock();
		lockDrawAllWin->unlock();
	}
#endif
        if( ENV.getBool(Env::isRunning) && _isThreadWorking )
        {
//            cout << "Drawing and wait is " << waitDrawAllWin << endl;
//            cout << "Drawing : g3d_draw " << paintNum++ << endl;
            g3d_draw();
        }
#ifdef WITH_XFORMS
	if (waitDrawAllWin != 0)
	{
//                cout << "All awake" << endl;
		waitDrawAllWin->wakeAll();
	}
#endif
	glPopMatrix();

	/*cout << "paintGL() : " << paintNum++ << endl;
	 cout << "------------------------------------------------" << endl;
	 cout << Xc[0] << " " << Xc[1] << " " << Xc[2] << endl;
	 cout << Xw[0] << " " << Xw[1] << " " << Xw[2] << endl;
	 cout << up[0] << " " << up[1] << " " << up[2] << endl;*/
}

void GLWidget::paintNewGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();

	p3d_vector4 Xc, Xw;
	p3d_vector4 up;

	//	computeNewVectors(Xc,Xw,up);

	qt_calc_cam_param();

	Xc[0] = JimXc[0];
	Xc[1] = JimXc[1];
	Xc[2] = JimXc[2];

	Xw[0] = JimXw[0];
	Xw[1] = JimXw[1];
	Xw[2] = JimXw[2];

	up[0] = Jimup[0];
	up[1] = Jimup[1];
	up[2] = Jimup[2];

	gluLookAt(Xc[0], Xc[1], Xc[2], Xw[0], Xw[1], Xw[2], up[0], up[1], up[2]);

	if ((lockDrawAllWin != 0) && (waitDrawAllWin != 0))
	{
		lockDrawAllWin->lock();
		lockDrawAllWin->unlock();
	}

	g3d_draw();

	if (waitDrawAllWin != 0)
	{
		waitDrawAllWin->wakeAll();
	}

	glPopMatrix();

	/*cout << "paintGL() : " << paintNum++ << endl;
	 cout << "------------------------------------------------" << endl;
	 cout << Xc[0] << " " << Xc[1] << " " << Xc[2] << endl;
	 cout << Xw[0] << " " << Xw[1] << " " << Xw[2] << endl;
	 cout << up[0] << " " << up[1] << " " << up[2] << endl;*/
}
void GLWidget::resizeGL(int width, int height)
{
	glViewport(0, 0, (GLint) width, (GLint) height);
//	qglClearColor(QColor::fromCmykF(0.0, 0.0, 0.0, 0.0));

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(40.0, (GLdouble) width / (GLdouble) height, size / 1000.,
			size * 1000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void GLWidget::computeNewVectors(p3d_vector4& Xc, p3d_vector4& Xw,
		p3d_vector4& up)
{
	static p3d_matrix4 Txc =
	{
	{ 1, 0, 0, 0 },
	{ 0, 1, 0, 0 },
	{ 0, 0, 1, 0 },
	{ 0, 0, 0, 1 } };
	p3d_matrix4 Id =
	{
	{ 1, 0, 0, 0 },
	{ 0, 1, 0, 0 },
	{ 0, 0, 1, 0 },
	{ 0, 0, 0, 1 } };
	p3d_matrix4 m_aux;
	p3d_vector4 Xx;

	Xx[0] = x;
	Xx[1] = y;
	Xx[2] = z;
	Xx[3] = 1.0;
	p3d_matvec4Mult(Id, Xx, Xw);

	Txc[0][3] = this->zo * (cos(this->az) * cos(this->el));
	Txc[1][3] = this->zo * (sin(this->az) * cos(this->el));
	Txc[2][3] = this->zo * sin(this->el);

	p3d_mat4Mult(Id, Txc, m_aux);
	p3d_matvec4Mult(m_aux, Xx, Xc);

	p3d_matvec4Mult(Id, this->up, up);
}

int _i;
int _j;

void qt_get_win_mouse(int* i, int *j)
{
	*i = _i;
	*j = _j;
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	if (!_light)
	{
		_i = event->x();// - lastPos.x());
		_j = event->y();// - lastPos.y());
		qt_canvas_viewing(1, 0);
		//		_watingMouseRelease = true;
	}

	lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	if (!_light)
	{
		_i = event->x();
		_j = event->y();

                if ((event->buttons() & Qt::LeftButton)&&(mouse_mode==0))
		{
			qt_canvas_viewing(0, 0);
		}
                else if ((event->buttons() & Qt::MidButton) || ((event->buttons() & Qt::LeftButton)&&(mouse_mode==1)) )
		{
			qt_canvas_viewing(0, 1);
		}
                else if ((event->buttons() & Qt::RightButton) || ((event->buttons() & Qt::LeftButton)&&(mouse_mode==2)) )
		{
			qt_canvas_viewing(0, 2);
		}
		_watingMouseRelease = true;
	}
	else
	{
		int dx = event->x() - lastPos.x();
		int dy = event->y() - lastPos.y();

		double xmin, xmax, ymin, ymax, zmin, zmax;
		p3d_get_env_box(&xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

		if (event->buttons() & Qt::LeftButton)
		{
			if( (dx<0 && qt_get_cur_g3d_win()->lightPosition[0]>xmin )
					|| (dx>0 && qt_get_cur_g3d_win()->lightPosition[0]>xmax ) )
			{
				qt_get_cur_g3d_win()->lightPosition[0] += (dx*(xmax-xmin) / 100);
			}
		}
		else if (event->buttons() & Qt::MidButton)
		{
			if( (dy<0 && qt_get_cur_g3d_win()->lightPosition[1]>ymin )
								|| (dy>0 && qt_get_cur_g3d_win()->lightPosition[1]>ymax ) )
			{
				qt_get_cur_g3d_win()->lightPosition[1] += (dy * (ymax-ymin ) / 100);
			}
		}
		else if (event->buttons() & Qt::RightButton)
		{
			if( (dy<0 && qt_get_cur_g3d_win()->lightPosition[2]>zmin )
											|| (dy>0 && qt_get_cur_g3d_win()->lightPosition[2]>zmax ) )
			{
				qt_get_cur_g3d_win()->lightPosition[2] += (dy*(zmax - zmin) / 100);
			}
		}
	}
	//	updateGL();

	pipe2openGl->updatePipe();

	//	cout << "i = " << event->x() << endl;
	//	cout << "j = " << event->y() << endl;

	/*static double x,y,z,zo,el,az;

	 double x_aux,y_aux,az_aux,rotinc,incinc;

	 x=this->x;
	 y=this->y;
	 z=this->z;
	 zo=this->zo;
	 el=this->el;
	 az=this->az;

	 cout << "this->x = " << x << endl;
	 cout << "this->y = " << y << endl;
	 cout << "this->z = " << z << endl;

	 cout << "this->zo = " << zo << endl;
	 cout << "this->el = " << el << endl;
	 cout << "this->az = " << az << endl << endl;

	 if (event->buttons() & Qt::LeftButton)
	 {
	 this->zo = (zo * i) / w + zo;
	 if (this->zo < .0)
	 this->zo = .0;
	 }
	 else if (event->buttons() & Qt::RightButton)
	 {
	 this->az = (-2 * GAIN_AZ * i) / w + az;
	 if (this->az < .0)
	 this->az = 2 * M_PI;
	 if (this->az > 2 * M_PI)
	 this->az = .0;
	 this->el = (GAIN_EL * j) / w + el;
	 if (this->el < -M_PI / 2.0)
	 this->el = -M_PI / 2.0;
	 if (this->el > M_PI / 2.0)
	 this->el = M_PI / 2.0;
	 }
	 else if (event->buttons() & Qt::MidButton)
	 {
	 rotinc = (-GAIN_AZ * i) / w;
	 incinc = (-GAIN_EL * j) / w;
	 //		rotation effects
	 this->x = x + 2.0 * zo * cos(el) * sin(rotinc / 2.0) * sin(az
	 + rotinc / 2.0);
	 this->y = y - 2.0 * zo * cos(el) * sin(rotinc / 2.0) * cos(az
	 + rotinc / 2.0);
	 this->az = az + rotinc;

	 if (this->az < .0)
	 this->az = 2 * M_PI;
	 if (this->az > 2 * M_PI)
	 this->az = .0;
	 x_aux = this->x;
	 y_aux = this->y;
	 az_aux = this->az;
	 //		incline effects
	 this->z = z - 2.0 * zo * sin(incinc / 2.0) * cos(el + incinc / 2.0);
	 this->x = x_aux + 2.0 * zo * sin(incinc / 2.0) * sin(el + incinc
	 / 2.0) * cos(az_aux);
	 this->y = y_aux + 2.0 * zo * sin(incinc / 2.0) * sin(el + incinc
	 / 2.0) * sin(az_aux);
	 this->el = el + incinc;
	 if (this->el < -M_PI / 2)
	 this->el = -M_PI / 2;
	 if (this->el > M_PI / 2)
	 this->el = M_PI / 2;
	 }*/
}

void GLWidget::keyPressEvent(QKeyEvent *e)
{
	cout << "A key is pressed" << endl;

	switch (e->key())
	{
	case Qt::Key_Shift:
		_light = !_light;
		cout << "Shift pressed" << endl;
		break;

	case Qt::Key_Plus:
		if (qt_get_cur_g3d_win()->shadowContrast < 9.94)
		{
			qt_get_cur_g3d_win()->shadowContrast += 0.05;
		}
		pipe2openGl->updatePipe();
		cout << "+ pressed" << endl;
		break;

	case Qt::Key_Minus:
		if (qt_get_cur_g3d_win()->shadowContrast > 0.06)
		{
			qt_get_cur_g3d_win()->shadowContrast -= 0.05;
		}
		pipe2openGl->updatePipe();
		cout << "- pressed" << endl;
		break;

         case Qt::Key_A:
                cout << "A pressed" << endl;
                break;

         case Qt::Key_B:
                cout << "B pressed" << endl;
                break;
	}
}

void GLWidget::keyReleaseEvent(QKeyEvent *e)
{
	cout << "A key is release" << endl;

	switch (e->key())
	{

	break;
	}
}

void GLWidget::normalizeAngle(int *angle)
{
	while (*angle < 0)
		*angle += 360 * 160;
	while (*angle > 360 * 160)
		*angle -= 360 * 160;
}
