#include "qtESTWindow.hpp"
//#include "c_to_qt.h"

#include <iostream>
//#include "../../planner/Diffusion_cxx/EST.hpp"

using namespace std;

qtESTWindow::qtESTWindow() : qtBaseWindow() {

	string = tr("EST");

	box->setTitle("EST Planner");

	spacer = new QWidget();
	spacer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);

	init();
}

// Constructor
void qtESTWindow::init()
{
	// Buttons And sliders-------------------------------------------------------------------
	QPushButton* EST = new QPushButton("EST Planner");
	connect(EST, SIGNAL(clicked()),this, SLOT(ESTPlan()),Qt::DirectConnection);

//	LabeledSlider* nbESTTraj = createSlider(tr("Number of trajectories"), Env::nbESTTraj, 1, 10 );
//	LabeledDoubleSlider* minStep = createDoubleSlider(tr("Min step"), Env::MinStep, 0, 100 );

//	numberIterations->setValue(ENV.getInt(Env::nbCostOptimize));

//	QCheckBox* isDebug = createCheckBox(tr("Debug"), Env::debugCostOptim);

	// Connection to Layout------------------------------------------------------------------
	Layout->addWidget(EST,1,0);
//	Layout->addWidget(nbESTTraj,2,0);
//	Layout->addWidget(numberIterations,3,0);
//	Layout->addWidget(maxFactor,4,0);
//	Layout->addWidget(minStep,5,0);
//	Layout->addWidget(isDebug,6,0);
	Layout->addWidget(spacer);


}

void qtESTWindow:: ESTPlan() {
//	  runButton->setDisabled(true);
//	  resetButton->setDisabled(true);
	  std::string str = "p3d_RunEST";
//	  write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

qtESTWindow::~qtESTWindow()
{
//	delete box;
//	delete Layout;
}

#include "moc_qtESTWindow.cpp"
