#include "qtColisionTestWindow.hpp"
//#include "c_to_qt.h"

using namespace std;

qtColisionTestWindow::qtColisionTestWindow() :
	qtBaseWindow() {

	string= tr("Test KChecking");

	box->setTitle("Test Models");

	spacer = new QWidget();
	spacer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);

	init();

//	TestModel context();
//	tests = context;
}

// Constructor
void qtColisionTestWindow::init()
{
	QPushButton* computeColisionTests = new QPushButton("Collisions");
	connect(computeColisionTests, SIGNAL(clicked()),this, SLOT(colisions()));

	QPushButton* computeLPTests = new QPushButton("Localpaths");
	connect(computeLPTests, SIGNAL(clicked()),this, SLOT(localpaths()),Qt::DirectConnection);

	QPushButton* computeAllTests = new QPushButton("AllTests");
	connect(computeAllTests, SIGNAL(clicked()),this, SLOT(allTests()),Qt::DirectConnection);

	QPushButton* computeDistances = new QPushButton("Distance");
	connect(computeDistances, SIGNAL(clicked()),this, SLOT(distance()),Qt::DirectConnection);

	//Adding to layer---------------------------------------------------------------------------
	//------------------------------------------------------------------------------------------
	Layout->addWidget(computeColisionTests,0,0);
	Layout->addWidget(computeLPTests,1,0);
	Layout->addWidget(computeAllTests,2,0);
	Layout->addWidget(computeDistances,3,0);
	Layout->addWidget(spacer);

}

void qtColisionTestWindow::distance(){
	cout << "Tests of distance computation" << endl;
	tests = new TestModel();
	tests->distEnv();
	delete tests;
}

void qtColisionTestWindow::colisions(){
	tests = new TestModel();
	cout <<  tests->nbOfColisionsPerSeconds() << " collisions per second" << endl;
	delete tests;
}

void qtColisionTestWindow::localpaths(){
	tests = new TestModel();
	cout << tests->nbOfLocalPathsPerSeconds() << " localpaths per second" << endl;
	delete tests;
}

void qtColisionTestWindow::allTests(){
	tests = new TestModel();
	tests->runAllTests();
	delete tests;
}

qtColisionTestWindow::~qtColisionTestWindow()
{
//	delete box;
//	delete Layout;
}

#include "moc_qtColisionTestWindow.cpp"
