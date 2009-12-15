#include "qtHriWindow.hpp"
#include "Planner-pkg.h"

#include "../../planner_cxx/HRI_CostSpace/HRICS_HAMP.h"
#include "../../planner_cxx/HRI_CostSpace/HRICS_old.h"

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

    // Akin's functions
    akinBox = new QVGroupBox(tr("Hri Space (akin)"));

    whichTestBox = new QComboBox();
    whichTestBox->insertItem(0, "Distance");
    whichTestBox->insertItem(1, "Comfort");
    whichTestBox->insertItem(2, "Visibility");
    whichTestBox->insertItem(3, "Combined");
    whichTestBox->setCurrentIndex((int)0);
    whichTestBox->setDisabled(false);

    connect(whichTestBox, SIGNAL(currentIndexChanged(int)),this, SLOT(setWhichTestSlot(int)), Qt::DirectConnection);

    LabeledSlider* JointSlider = createSlider(tr("Joint Id"), Env::akinJntId, 0., 50.);

    LabeledDoubleSlider* distanceSlider = createDoubleSlider(tr("Distance"), Env::Kdistance, 0., 50.);
    LabeledDoubleSlider* visibilitySlider = createDoubleSlider(tr("Visibility"), Env::Kvisibility, 0., 50.);
    LabeledDoubleSlider* visThresh = createDoubleSlider(tr("Threshold Visib."), Env::visThresh, 0., 50.);

    QPushButton* enableAkin = new QPushButton("Enable Akin Function");
    connect(enableAkin, SIGNAL(clicked()),this, SLOT(enableHriSpace()));

    akinBox->addWidget(JointSlider);
    akinBox->addWidget(distanceSlider);
    akinBox->addWidget(visibilitySlider);
    akinBox->addWidget(visThresh);

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


    QPushButton* changeColor = new QPushButton("Change Color");
    connect(changeColor, SIGNAL(clicked()),this, SLOT(changeColor()));


    // Connection to Layout
    int Row(0);
    Layout->addWidget(akinBox);

    Layout->addWidget(useHriDis/*, Row, 0*/);
    Layout->addWidget(useHriPen/*, Row++, 1*/);
    Layout->addWidget(useHriNat/*, Row++, 1*/);
    Layout->addWidget(enable/*, Row++, 1*/);
    Layout->addWidget(zoneSize);

    Layout->addWidget(changeColor);

    akinBox->addWidget(enableAkin);
    akinBox->addWidget(whichTestBox);


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

void qtHriWindow::setWhichTestSlot(int test)
{
#ifdef HRI_PLANNER
    hriSpace->changeTest(test);
    cout << "Change test to :" << test << endl;
#else
    cout << "HRI Planner not compiled nor linked" << endl;
#endif
}

void qtHriWindow::enableHriSpace(void)
{
#ifdef HRI_PLANNER
    if(hriSpace)
    {
        delete hriSpace;
    }
    hriSpace = new HRICS::HriSpaceCost(XYZ_ROBOT,ENV.getInt(Env::akinJntId));
#else
    cout << "HRI Planner not compiled nor linked" << endl;
#endif

    ENV.setBool(Env::isCostSpace,true);
    ENV.setBool(Env::enableHri,true);
    ENV.setBool(Env::isHriTS,true);
    cout << "Env::enableHri is set to true, joint number is :"<< ENV.getInt(Env::akinJntId) << endl;
    cout << "Robot is :" << XYZ_ROBOT->name << endl;
    whichTestBox->setDisabled(false);
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
