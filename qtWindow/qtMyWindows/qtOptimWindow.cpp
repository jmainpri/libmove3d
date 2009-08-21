#include "qtOptimWindow.hpp"
#include <iostream>
#include "Planner-pkg.h"
//#include "c_to_qt.h"

qtOptimWindow::qtOptimWindow() : qtBaseWindow() {

	string= tr("Optim");

	box->setTitle("Optimization");

	spacer = new QWidget();
	spacer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);

	init();
}

// Constructor
void qtOptimWindow::init()
{
	QPushButton* computeGridAndExtract = new QPushButton("2D Best Path");
	connect(computeGridAndExtract, SIGNAL(clicked()),this, SLOT(computeGridAndExtract()));

//	LabeledSlider* numberIterations = createSlider(tr("Number of iteration"), Env::nbCostOptimize, 0, 5000 );

	QPushButton* optimize = new QPushButton("Optimize");
	connect(optimize, SIGNAL(clicked()),this, SLOT(optimizeCost()),Qt::DirectConnection);

	QPushButton* shortCut = new QPushButton("ShortCut");
	connect(shortCut, SIGNAL(clicked()),this, SLOT(shortCutCost()),Qt::DirectConnection);

//	LabeledDoubleSlider* step = createDoubleSlider(tr("Step"), Env::MinStep, 0, 100 );

//	QCheckBox* isDebug = createCheckBox(tr("Debug"), Env::debugCostOptim);

	//Adding to layer---------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------
	Layout->addWidget(computeGridAndExtract,0,0);
//	Layout->addWidget(numberIterations,1,0);
//	Layout->addWidget(step,2,0);
	Layout->addWidget(optimize,3,0);
//	Layout->addWidget(isDebug,4,0);
	Layout->addWidget(shortCut,5,0);
	Layout->addWidget(spacer);

}

void qtOptimWindow::computeGridAndExtract()
{
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_CreateDenseRoadmap(robotPt);
	p3d_ExtractBestTraj(XYZ_GRAPH);
	ENV.setBool(Env::drawGraph,false);
//	ENV.setBool(Env::drawTraj,true);
}

void qtOptimWindow::computeGrid(){
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_CreateDenseRoadmap(robotPt);
}

void qtOptimWindow::optimizeCost()
{
	std::string str = "optimize";
//	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void qtOptimWindow::shortCutCost()
{
	std::string str = "shortCut";
//	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}


void qtOptimWindow::extractBestTraj()
{
	p3d_ExtractBestTraj(XYZ_GRAPH);
}

qtOptimWindow::~qtOptimWindow()
{
//	delete box;
//	delete Layout;
}

#include "moc_qtOptimWindow.cpp"
