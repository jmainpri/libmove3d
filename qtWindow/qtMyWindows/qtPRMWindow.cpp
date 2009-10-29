#include "qtPRMWindow.hpp"
#include "../cppToQt.hpp"

using namespace std;

qtPRMWindow::qtPRMWindow() : qtBaseWindow()
{
	string= tr("PRM");
	box->setTitle("PRM Planner");
	init();
}

// Constructor
void qtPRMWindow::init()
{
	QPushButton* runPRM = new QPushButton("Run PRM");
	connect(runPRM, SIGNAL(clicked()),this, SLOT(runPRMAlgo()));
	Layout->addWidget(runPRM);
}

void qtPRMWindow::runPRMAlgo()
{
	cout << "Running PRM pipe" << endl;
	std::string str = "runPRM";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

qtPRMWindow::~qtPRMWindow()
{
//	delete box;
//	delete Layout;
}

#include "moc_qtPRMWindow.cpp"
