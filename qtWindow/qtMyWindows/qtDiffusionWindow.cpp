#include "qtDiffusionWindow.hpp"
#include <iostream>
#include "Planner-pkg.h"

qtDiffusionWindow::qtDiffusionWindow() : qtBaseWindow() {

	string= tr("Diffusion");
	init();
}

// Constructor
void qtDiffusionWindow::init()
{


	    // Diffusion - general
   	QCheckBox* biDirCheckBox = createCheckBox(tr("&Bidirectional"), Env::biDir);
    QCheckBox* expandToGoalCheckBox = createCheckBox(tr("With &goal"), Env::expandToGoal);
    QCheckBox* expandBalancedCheckBox = createCheckBox(tr("Balan&ced"), Env::expandBalanced);
    QCheckBox* expandControlCheckBox = createCheckBox(tr("E&xpand control"), Env::expandControl);
    QCheckBox* discardNodesCheckBox = createCheckBox(tr("&Discard nodes"), Env::discardNodes);
    QCheckBox* isManhattanCheckBox = createCheckBox(tr("ML-RRT"), Env::isManhattan);
//    QCheckBox* isCostEnvCheckBox = createCheckBox(tr("Cost Env"), Env::costEnv);


	// Diffusion - Expansion Process
	expansionProcessBox = new QVGroupBox("Tree expansion method");

	QCheckBox* addCyclesCheckBox = createCheckBox(tr("Add &cycles"), Env::addCycles);
//	QCheckBox* useRefiRadius = createCheckBox(tr("Set Manual Ref Radius"), Env::useRefiRadius);

	LabeledDoubleSlider* extensionStepSlider = createDoubleSlider("Extension step", Env::extensionStep, 1, 20);
//	LabeledDoubleSlider* refiRadiusSlider = createDoubleSlider("Refinement Radius", Env::refiRadius, 1, 300);

	QComboBox* expansionMethodBox = new QComboBox();
	expansionMethodBox->insertItem(0, "Extend");
	expansionMethodBox->insertItem(1, "Extend n steps");
	expansionMethodBox->insertItem(2, "Connect");
	expansionMethodBox->insertItem(3, "Cost Connect");

	expansionMethodBox->setCurrentIndex((int)ENV.getExpansionMethod());

	connect(expansionMethodBox, SIGNAL(currentIndexChanged(int)),&ENV, SLOT(setExpansionMethodSlot(int)), Qt::DirectConnection);
	connect(&ENV, SIGNAL(expansionMethodChanged(int)),expansionMethodBox, SLOT(setCurrentIndex(int)));

	expansionProcessBox->addWidget(addCyclesCheckBox);
	expansionProcessBox->addWidget(expansionMethodBox);
	expansionProcessBox->addWidget(extensionStepSlider);
//	expansionProcessBox->addWidget(useRefiRadius);
//	expansionProcessBox->addWidget(refiRadiusSlider);


    // Diffusion - Cost Spaces
    costSpacesBox = new QVGroupBox("Cost Spaces");
    LabeledDoubleSlider* temperatureRate = createDoubleSlider("Temperature rate", Env::temperatureRate, 0., 5.);
    LabeledDoubleSlider* initialTemperature = createDoubleSlider("initial Temperature", Env::initialTemperature, 0., 2.);
    LabeledDoubleSlider* alphaSlider = createDoubleSlider("Alpha", Env::alpha, 0., 1.);

    LabeledSlider* maxCostOptimFailuresSlider = createSlider("maxCostOptimFailures", Env::maxCostOptimFailures, 10, 1000);

    QPushButton* saveCostTemperature = new QPushButton("Save cost and temperature");
    connect(saveCostTemperature, SIGNAL(clicked()),this, SLOT(saveCostTemperature()));
//    connect(ENV.getObject(Env::costEnv), SIGNAL(valueChanged(bool)), this, SLOT(costEnv()));

//    costSpacesBox->addWidget(isCostEnvCheckBox);
	costSpacesBox->addWidget(initialTemperature);
	costSpacesBox->addWidget(temperatureRate);
	costSpacesBox->addWidget(alphaSlider);
	costSpacesBox->addWidget(maxCostOptimFailuresSlider);
	costSpacesBox->addWidget(saveCostTemperature);


	// Connection to Layout
	int Row(0);

	Layout->addWidget(expandToGoalCheckBox, Row, 0);
	Layout->addWidget(biDirCheckBox, Row++, 1);
	Layout->addWidget(expandControlCheckBox, Row, 0);
	Layout->addWidget(expandBalancedCheckBox, Row++, 1);
	Layout->addWidget(isManhattanCheckBox, Row, 0);
	Layout->addWidget(discardNodesCheckBox, Row++, 1);

	Layout->addWidget(expansionProcessBox, Row++, 0, 1, -1);
	Layout->addWidget(costSpacesBox, Row++, 0, 1, -1);

	spacer = new QWidget();
	spacer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);
	Layout->addWidget(spacer);
}


void qtDiffusionWindow::costEnv(){
	//p3d_SetIsCostFuncSpace(!p3d_GetIsCostFuncSpace());
}


void qtDiffusionWindow::saveCostTemperature()
{
	int i = 0;
	int j = 0;

	FILE* traj_file = fopen("costTempChro.csv", "w");

	for (p3d_compco *comp = XYZ_GRAPH->comp; comp != NULL && j < 10; comp
			= comp->suiv, j++) {

		for (p3d_list_node *n = comp->dist_nodes; n != NULL && i < 10000; n
				= n->next, i++) {
			fprintf(traj_file, "%d, %5.6f, %5.20f\n", n->N->num, n->N->cost, n->N->temp);
		}
	}

	fclose(traj_file);
	PrintInfo(("'costTemChro.csv' creation\n"));
}


qtDiffusionWindow::~qtDiffusionWindow()
{
//	delete box;
//	delete Layout;
}

#include "moc_qtDiffusionWindow.cpp"
