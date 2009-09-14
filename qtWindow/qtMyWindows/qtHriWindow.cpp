#include "qtHriWindow.hpp"
#include "Planner-pkg.h"

#include "../../planner_cxx/HRICost/HriTaskSpaceCost.hpp"
#include "../../planner_cxx/HRICost/HriCost.hpp"

using namespace std;

qtHriWindow::qtHriWindow() : qtBaseWindow() {
	init();
}

// Constructor
void qtHriWindow::init()
{
	string= tr("Hri");

	//  general
	QCheckBox* useHriDis = createCheckBox(tr("useHriDis"), Env::useHriDis);
	QCheckBox* useHriPen = createCheckBox(tr("useHriPen"), Env::useHriPen);
	QCheckBox* useHriNat = createCheckBox(tr("useHriNat"), Env::useHriNat);
	QCheckBox* enable = createCheckBox(tr("enable"), Env::enableHri);

	// Cost Function Box
	zoneBox = new QVGroupBox(tr("Global Cost Function"));
	LabeledDoubleSlider* zoneSize = createDoubleSlider(tr("Zone Size"), Env::zone_size, 0., 300.);
	LabeledDoubleSlider* coeffPen = createDoubleSlider(tr("Penetration"), Env::coeffPen, 0., 100.);
	LabeledDoubleSlider* coeffDis = createDoubleSlider(tr("C-Distance"), Env::coeffDis, 0., 100.);
	LabeledDoubleSlider* coeffNat = createDoubleSlider(tr("Natural"), Env::coeffNat, 0., 100.);

	// Box Natural
	naturalBox = new QVGroupBox(tr("Natural Function"));
	LabeledDoubleSlider* coeffLim = createDoubleSlider(tr("Joint Limit"), Env::coeffLim, 0., 100.);
	LabeledDoubleSlider* coeffTas = createDoubleSlider(tr("T-Distance"), Env::coeffTas, 0., 100.);
	LabeledDoubleSlider* coeffHei = createDoubleSlider(tr("Height Weight"), Env::coeffHei, 0., 100.);

	QComboBox* whichTestBox = new QComboBox();
	whichTestBox->insertItem(0, "Distance");
	whichTestBox->insertItem(1, "Comfort");
	whichTestBox->insertItem(2, "Visibily");
	whichTestBox->insertItem(3, "Combined");

	// Buttons

//	QPushButton* computeCostTab = new QPushButton("Compute Cost Tab");
//	connect(computeCostTab, SIGNAL(clicked()),this, SLOT(computeCostTab()));
//	Layout->addWidget(computeCostTab);

//	QPushButton* computeCostGround = new QPushButton("Compute Cost Ground");
//	connect(computeCostGround, SIGNAL(clicked()),this, SLOT(computeCostGround()));
//	Layout->addWidget(computeCostGround);


//	QPushButton* computeFuncGround = new QPushButton("Compute Function Ground");
//	connect(computeFuncGround, SIGNAL(clicked()),this, SLOT(computeFunctionGround()));
//	Layout->addWidget(computeFuncGround);

	QPushButton* enableAkin = new QPushButton("Enable Akin Function");
	connect(enableAkin, SIGNAL(clicked()),this, SLOT(enableHriSpace()));
	Layout->addWidget(enableAkin);

	QPushButton* changeColor = new QPushButton("Change Color");
	connect(changeColor, SIGNAL(clicked()),this, SLOT(changeColor()));
	Layout->addWidget(changeColor);

	LabeledSlider* whichCostFunc = createSlider("which cost function", Env::test, 0, 2);

	// Connection to Layout
	int Row(0);
	Layout->addWidget(useHriDis/*, Row, 0*/);
	Layout->addWidget(useHriPen/*, Row++, 1*/);
	Layout->addWidget(useHriNat/*, Row++, 1*/);
	Layout->addWidget(enable/*, Row++, 1*/);
	Layout->addWidget(zoneSize);

	zoneBox->addWidget(coeffPen);
	zoneBox->addWidget(coeffDis);
	zoneBox->addWidget(coeffNat);

	naturalBox->addWidget(coeffLim);
	naturalBox->addWidget(coeffTas);
	naturalBox->addWidget(coeffHei);

	Layout->addWidget(zoneBox);
	Layout->addWidget(naturalBox);

	// Set Spacer
	QWidget* spacer = new QWidget();
	spacer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);
	Layout->addWidget(spacer);

}

void qtHriWindow::enableHriSpace(void)
{
	int idJnt = 13;
	hriSpace = new HriSpaceCost(XYZ_ROBOT,idJnt);
	ENV.setBool(Env::isCostSpace,true);
	ENV.setBool(Env::isHriTS,true);
	cout << "Env::enableHri is set to true, joint number is :"<< idJnt << endl;
	cout << "Robot is :" << XYZ_ROBOT->name << endl;
}

void qtHriWindow::computeCostTab(void)
{
	int size = 80;
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
//	confCost tabCost[size*size];
	//Hri zone(ENV.getDouble(Env::zone_size));

//	hri_zones.setCostToTab(robotPt,tabCost,size);
//	hri_zones.writeConfCostToCsv(tabCost,size);
}

void qtHriWindow::computeCostGround(void)
{
	int size = 50;
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
//	confCost tabCost[size*size];
	//Hri zone(ENV.getDouble(Env::zone_size));
//
//	hri_zones.setCostToTab(robotPt,tabCost,size);
//	hri_zones.writeConfCostToObPlane(tabCost,size);
}

void qtHriWindow::computeFunctionGround(void)
{
	int size = 50;
//	confCost tabCost[size*size];
//	hri_zones.costTabFormFunction(tabCost);
//	hri_zones.writeConfCostToObPlane(tabCost,size);
}

void qtHriWindow::changeColor(void)
{
	hri_zones.parseEnvForZone();
	hri_zones.changeColor();
	g3d_draw_allwin_active();
}

qtHriWindow::~qtHriWindow()
{
//	delete box;
//	delete Layout;
}

#include "moc_qtHriWindow.cpp"
