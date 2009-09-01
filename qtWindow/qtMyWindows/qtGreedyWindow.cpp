#include "qtGreedyWindow.hpp"
#include "../cppToQt.hpp"

using namespace std;

qtGreedyWindow::qtGreedyWindow() : qtBaseWindow() {

	string = tr("Greedy");

	box->setTitle("Greedy Planner");

	spacer = new QWidget();
	spacer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);

	init();
}

// Constructor
void qtGreedyWindow::init()
{
	// Buttons And sliders-------------------------------------------------------------------
	QPushButton* greedy = new QPushButton("Greedy Planner");
	connect(greedy, SIGNAL(clicked()),this, SLOT(greedyPlan()),Qt::DirectConnection);

	LabeledSlider* nbGreedyTraj = createSlider(tr("Number of trajectories"), Env::nbGreedyTraj, 1, 10 );
	LabeledSlider* numberIterations = createSlider(tr("Number of iteration"), Env::nbCostOptimize, 0, 5000 );
	LabeledDoubleSlider* maxFactor = createDoubleSlider(tr("Start Factor"), Env::MaxFactor, 0, 200 );
	LabeledDoubleSlider* minStep = createDoubleSlider(tr("Min step"), Env::MinStep, 0, 100 );

//	double dmax=0;
//	p3d_col_get_dmax(&dmax);
//
//	minStep->setValue(dmax);
	numberIterations->setValue(ENV.getInt(Env::nbCostOptimize));

	QCheckBox* isDebug = createCheckBox(tr("Debug"), Env::debugCostOptim);

	// Connection to Layout------------------------------------------------------------------
	Layout->addWidget(greedy,1,0);
	Layout->addWidget(nbGreedyTraj,2,0);
	Layout->addWidget(numberIterations,3,0);
	Layout->addWidget(maxFactor,4,0);
	Layout->addWidget(minStep,5,0);
	Layout->addWidget(isDebug,6,0);
	Layout->addWidget(spacer);


}

void qtGreedyWindow:: greedyPlan() {
//	  runButton->setDisabled(true);
//	  resetButton->setDisabled(true);
	  std::string str = "p3d_RunGreedy";
	  write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

qtGreedyWindow::~qtGreedyWindow()
{
//	delete box;
//	delete Layout;
}

#include "moc_qtGreedyWindow.cpp"
