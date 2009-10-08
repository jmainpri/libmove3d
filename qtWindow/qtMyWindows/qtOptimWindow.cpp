#include "qtOptimWindow.hpp"
#include <iostream>
#include "Planner-pkg.h"
#include "../cppToQt.hpp"

using namespace std;
using namespace tr1;

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

	LabeledSlider* numberIterations = createSlider(tr("Number of iteration"), Env::nbCostOptimize, 0, 5000 );

	QPushButton* optimize = new QPushButton("Cost Optimize");
	connect(optimize, SIGNAL(clicked()),this, SLOT(optimizeCost()),Qt::DirectConnection);

	QPushButton* shortCut = new QPushButton("Cost ShortCut");
	connect(shortCut, SIGNAL(clicked()),this, SLOT(shortCutCost()),Qt::DirectConnection);

	QPushButton* removeNodes = new QPushButton("Remove Redundant Nodes");
	connect(removeNodes, SIGNAL(clicked()),this, SLOT(removeRedundant()),Qt::DirectConnection);

	QPushButton* testGraphSearch = new QPushButton("Dijkstra");
	connect(testGraphSearch, SIGNAL(clicked()),this, SLOT(graphSearchTest()),Qt::DirectConnection);

	QComboBox* costCriterium = new QComboBox();
	costCriterium->insertItem(INTEGRAL, "Integral");
	costCriterium->insertItem(MECHANICAL_WORK, "Mechanical Work");
	costCriterium->setCurrentIndex((int)(INTEGRAL));

	connect(costCriterium, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostCriterium(int)), Qt::DirectConnection);

	LabeledDoubleSlider* step = createDoubleSlider(tr("Step"), Env::MinStep, 0, 100 );

	QCheckBox* isDebug = createCheckBox(tr("Debug"), Env::debugCostOptim);

	//Adding to layer---------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------
	Layout->addWidget(computeGridAndExtract,0,0);
	Layout->addWidget(numberIterations,1,0);
	Layout->addWidget(step,2,0);
	Layout->addWidget(optimize,3,0);
	Layout->addWidget(shortCut,4,0);
	Layout->addWidget(removeNodes,5,0);
	Layout->addWidget(isDebug,6,0);
	Layout->addWidget(testGraphSearch,7,0);
	Layout->addWidget(costCriterium,8,0);
	Layout->addWidget(spacer);
}

void qtOptimWindow::computeGridAndExtract()
{
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

	if(XYZ_GRAPH)
	{
		p3d_del_graph(XYZ_GRAPH);
	}

	p3d_CreateDenseRoadmap(robotPt);

	Graph* ptrGraph = new Graph(XYZ_GRAPH);

	shared_ptr<Configuration> Init = ptrGraph->getRobot()->getInitialPosition();
	shared_ptr<Configuration> Goal = ptrGraph->getRobot()->getGoTo();

	Dijkstra graphSearch(ptrGraph);

	Trajectory* traj = graphSearch.extractTrajectory(Init,Goal);

	cout << "-------------------------------" << endl;
	cout << "Trajectory Cost = "<< traj->cost() << endl;
	cout << "   nl = "<< traj->getNbPaths() << endl;
	cout << "   length = "<< traj->getRangeMax() << endl;

	ENV.setBool(Env::drawGraph,false);
	ENV.setBool(Env::drawTraj,true);

	traj->replaceP3dTraj();

	std::string str = "g3d_draw_allwin_active";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);

	delete traj;

}

void qtOptimWindow::computeGrid(){
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_CreateDenseRoadmap(robotPt);

}

void qtOptimWindow::optimizeCost()
{
	std::string str = "optimize";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void qtOptimWindow::shortCutCost()
{
	std::string str = "shortCut";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void qtOptimWindow::removeRedundant()
{
	std::string str = "removeRedunantNodes";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void qtOptimWindow::graphSearchTest()
{
	std::string str = "graphSearchTest";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}


void qtOptimWindow::extractBestTraj()
{
	p3d_ExtractBestTraj(XYZ_GRAPH);
}

void qtOptimWindow::setCostCriterium(int choise) {
	p3d_SetDeltaCostChoice(choise);
}

qtOptimWindow::~qtOptimWindow()
{
//	delete box;
//	delete Layout;
}

#include "moc_qtOptimWindow.cpp"
