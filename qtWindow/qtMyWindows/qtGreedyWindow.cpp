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
	LabeledSlider* hight = createSlider(tr("Height Factor"), Env::heightFactor, 1, 10 );
	LabeledSlider* numberIterations = createSlider(tr("Number of iteration"), Env::nbCostOptimize, 0, 500 );
	LabeledDoubleSlider* maxFactor = createDoubleSlider(tr("Start Factor"), Env::MaxFactor, 0, 2000 );
	LabeledDoubleSlider* minStep = createDoubleSlider(tr("Min step"), Env::MinStep, 0, 1000 );
	LabeledDoubleSlider* costStep = createDoubleSlider(tr("Cost Step"), Env::CostStep, 0.01, 10 );

//	double dmax=0;
//	p3d_col_get_dmax(&dmax);
//
//	minStep->setValue(dmax);
	numberIterations->setValue(ENV.getInt(Env::nbCostOptimize));

	QCheckBox* isDebug = createCheckBox(tr("Debug"), Env::debugCostOptim);
	QCheckBox* recompute = createCheckBox(tr("Recompute All Traj Cost"), Env::trajCostRecompute);
	QCheckBox* withShortCut = createCheckBox(tr("Short Cut Rounds"), Env::withShortCut);
	QCheckBox* useTRRT = createCheckBox(tr("useTRRT"), Env::useTRRT);

	QPushButton* biasPos = new QPushButton("Biased Pos");
	connect(biasPos, SIGNAL(clicked()),this, SLOT(biasPos()),Qt::DirectConnection);

	QComboBox* expansionMethodBox = new QComboBox();
	expansionMethodBox->insertItem(0, "Extend");
	expansionMethodBox->insertItem(1, "Extend n steps");
	expansionMethodBox->insertItem(2, "Connect");
	expansionMethodBox->insertItem(3, "Cost Connect");
	expansionMethodBox->setCurrentIndex((int)ENV.getExpansionMethod());

	connect(expansionMethodBox, SIGNAL(currentIndexChanged(int)),&ENV, SLOT(setExpansionMethodSlot(int)), Qt::DirectConnection);
	connect(&ENV, SIGNAL(expansionMethodChanged(int)),expansionMethodBox, SLOT(setCurrentIndex(int)));

	QComboBox* costCriterium = new QComboBox();
	costCriterium->insertItem(INTEGRAL, "Integral");
	costCriterium->insertItem(MECHANICAL_WORK, "Mechanical Work");
	costCriterium->insertItem(VISIBILITY, "Visibility");
	costCriterium->setCurrentIndex((int)(INTEGRAL));

	connect(costCriterium, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostCriterium(int)), Qt::DirectConnection);

	// Connection to Layout------------------------------------------------------------------
	Layout->addWidget(greedy,1,0);
	Layout->addWidget(nbGreedyTraj,2,0);
	Layout->addWidget(numberIterations,3,0);
	Layout->addWidget(hight,4,0);
	Layout->addWidget(maxFactor,5,0);
	Layout->addWidget(minStep,6,0);
	Layout->addWidget(useTRRT,7,0);
	Layout->addWidget(isDebug,8,0);
	Layout->addWidget(recompute,9,0);
	Layout->addWidget(withShortCut,10,0);
	Layout->addWidget(expansionMethodBox,11,0);
	Layout->addWidget(expansionMethodBox,12,0);
	Layout->addWidget(costCriterium,13,0);
	Layout->addWidget(biasPos,14,0);
	Layout->addWidget(spacer);

}

void qtGreedyWindow::biasPos() {
        Robot* R = new Robot(XYZ_ROBOT);
	CostOptimization* costOptim = new CostOptimization(R,R->getTrajStruct());
	tr1::shared_ptr<Configuration> q = costOptim->cheat();
	costOptim->getRobot()->setAndUpdate(*q);
	std::string str = "g3d_draw_allwin_active";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void qtGreedyWindow:: greedyPlan() {
//	  runButton->setDisabled(true);
//	  resetButton->setDisabled(true);
	  std::string str = "p3d_RunGreedy";
	  write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void qtGreedyWindow:: setCostCriterium(int choise) {
	p3d_SetDeltaCostChoice(choise);
}

qtGreedyWindow::~qtGreedyWindow()
{
//	delete box;
//	delete Layout;
}

#include "moc_qtGreedyWindow.cpp"
