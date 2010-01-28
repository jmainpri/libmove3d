#include "qtVisuWindow.hpp"

qtVisuWindow::qtVisuWindow() : qtBaseWindow() {
	init();
}

// Constructor
void qtVisuWindow::init()
{
	string= tr("Visu");

	box->setTitle("Visualize");

	//  general
//	QCheckBox* temperature = createCheckBox(tr("Temp"), Env::printTemp);
//	QCheckBox* radius = createCheckBox(tr("Radius of refinement"), Env::printRadius);
//	QCheckBox* nbQRand = createCheckBox(tr("Nb QRand (Expansions)"), Env::printNbQRand);
//	QCheckBox* CollFail = createCheckBox(tr("Nb Collision Fail"), Env::printCollFail);
//	QCheckBox* CostFail = createCheckBox(tr("Nb Cost Fail"), Env::printCostFail);

	QLabel* Ltemperature = new QLabel;
	QLabel* Lradius = new QLabel;
	QLabel* LnbQRand = new QLabel;
	QLabel* LCollFail = new QLabel;
	QLabel* LCostFail = new QLabel;

	Ltemperature->setAlignment(Qt::AlignRight);
	Lradius->setAlignment(Qt::AlignRight);
	LnbQRand->setAlignment(Qt::AlignRight);
	LCollFail->setAlignment(Qt::AlignRight);
	LCostFail->setAlignment(Qt::AlignRight);

//	connect(ENV.getObject(Env::temperature), SIGNAL(valueChanged(double)),Ltemperature, SLOT(setNum(double)));//, Qt::DirectConnection);
//	connect(ENV.getObject(Env::nbQRand), SIGNAL(valueChanged(int)),LnbQRand, SLOT(setNum(int)));//, Qt::DirectConnection);
//	connect(ENV.getObject(Env::nbCostTransFailed), SIGNAL(valueChanged(int)),LCostFail, SLOT(setNum(int)));//, Qt::DirectConnection);
//	connect(ENV.getObject(Env::nbCollExpanFailed), SIGNAL(valueChanged(int)),LCollFail, SLOT(setNum(int)));//, Qt::DirectConnection);

	// Buttons
	QPushButton* saveStat = new QPushButton("Save Stat To File");
	connect(saveStat, SIGNAL(clicked()),this,SLOT(saveStat()));

	QPushButton* showStat = new QPushButton("Show Stat");
	connect(showStat, SIGNAL(clicked()),this,SLOT(showStat()));

	QPushButton* reset = new QPushButton("Reset");
	connect(reset, SIGNAL(clicked()),this,SLOT(resetCounters()));

	// Connection to Layout
	int Row(0);
//	Layout->addWidget(temperature,	1,0);
//	Layout->addWidget(radius,	2,0);
//	Layout->addWidget(nbQRand,	3,0);
//	Layout->addWidget(CollFail,	4,0);
//	Layout->addWidget(CostFail,	5,0);

	Layout->addWidget(Ltemperature,	1,1);
	Layout->addWidget(Lradius,	2,1);
	Layout->addWidget(LnbQRand,	3,1);
	Layout->addWidget(LCollFail,	4,1);
	Layout->addWidget(LCostFail,	5,1);

	Layout->addWidget(reset,6,1);
	// Set Spacer
	QWidget* spacer = new QWidget();
	spacer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);
	Layout->addWidget(spacer);

}

void qtVisuWindow::saveStat(){

}

void qtVisuWindow::showStat(){

}

void qtVisuWindow::resetCounters(){
//	ENV.setDouble(Env::temperature,0);
//	ENV.setInt(Env::nbQRand,0);
//	ENV.setInt(Env::nbCostTransFailed,0);
//	ENV.setInt(Env::nbCollExpanFailed,0);
}

qtVisuWindow::~qtVisuWindow()
{
//	delete box;
//	delete Layout;
}
