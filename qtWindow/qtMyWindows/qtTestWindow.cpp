#include "qtTestWindow.hpp"

#include <iostream>
#include <string>

#include "p3d/env.hpp"
#include "../cppToQt.hpp"

qtTestWindow::qtTestWindow() : qtBaseWindow()
{
	init();
}

// Constructor
void qtTestWindow::init()
{
	string = tr("Test Context");


//	LabeledDoubleSlider* multCost = createDoubleSlider(tr("Mult cost"), Env::multCost, 0, 100);

	// Buttons
	QPushButton* saveContext = new QPushButton("Add Context to Stack");
	connect(saveContext, SIGNAL(clicked()),this,SLOT(saveContext()));

	QPushButton* printSelectedContext = new QPushButton("Print Selected Context");
	connect(printSelectedContext, SIGNAL(clicked()),this,SLOT(printContext()));

	QPushButton* printAllContext = new QPushButton("Print All Contexts");
	connect(printAllContext, SIGNAL(clicked()),this,SLOT(printAllContext()));

	QPushButton* resetContext = new QPushButton("Reset Context Stack");
	connect(resetContext, SIGNAL(clicked()),this,SLOT(resetContext()));

	QPushButton* setSelectedContext = new QPushButton("Set Selected Context");
	connect(setSelectedContext, SIGNAL(clicked()),this,SLOT(setToSelected()));

	runAllRoundsRRT = new QPushButton("Run Multi RRT");
	connect(runAllRoundsRRT, SIGNAL(clicked()),this,SLOT(runAllRRT()));

	runAllRoundsGreedy = new QPushButton("Run Multi Greedy");
	connect(runAllRoundsGreedy, SIGNAL(clicked()),this,SLOT(runAllGreedy()));

	LabeledSlider* nbTest = createSlider(tr("Nb Of rounds"), Env::nbRound, 0, 50);

	QPushButton* showHisto = new QPushButton("Show Histograme");
	connect(showHisto, SIGNAL(clicked()),this, SLOT(showHistoWindow()));

	nameEdit = new QLineEdit;
	QLabel* StringLabel = new QLabel(tr("Name Of Context:"));

	contextList = new QListWidget;

	// Connection to Layout
	int Row(0);

	Layout->addWidget(StringLabel);
	Layout->addWidget(nameEdit);
	Layout->addWidget(saveContext);
	Layout->addWidget(resetContext);
	Layout->addWidget(printAllContext);
	Layout->addWidget(contextList);
	Layout->addWidget(printSelectedContext);
	Layout->addWidget(setSelectedContext);
	Layout->addWidget(nbTest);
	Layout->addWidget(runAllRoundsRRT);
	Layout->addWidget(runAllRoundsGreedy);
	Layout->addWidget(showHisto);


	// Set Spacer
	QWidget* spacer = new QWidget();
	spacer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);
	Layout->addWidget(spacer);

}

void qtTestWindow::saveContext()
{
	ENV.setString(Env::nameOfFile,nameEdit->text().toStdString());

	QListWidgetItem* item= new QListWidgetItem(contextList);
	itemList.push_back(item);
	itemList.back()->setText(nameEdit->text());

	storedContext.saveCurrentEnvToStack();
}

void qtTestWindow::printAllContext()
{
	if( storedContext.getNumberStored()>0){

		for(uint i=0;i<storedContext.getNumberStored();i++){
			std::cout << "------------ Context Number " << i << " ------------" << std::endl;
			storedContext.printData(i);
		}
		std::cout << "-------------------------------------------" << std::endl;
		std::cout << " Total number of contexts in stack =  " << storedContext.getNumberStored() << std::endl;
		std::cout << "-------------------------------------------" << std::endl;
	}
	else{
		std::cout << "Warning: no context in stack" << std::endl;
	}
}

void qtTestWindow::printContext()
{
	if( storedContext.getNumberStored() > 0 )
	{
		int i =  contextList->currentRow();
		std::cout << "------------ Context Number " << i << " ------------" << std::endl;
		storedContext.printData(i);
	}
	else
	{
		std::cout << "Warning: no context in stack" << std::endl;
	}
}

void qtTestWindow::setToSelected()
{
	if( storedContext.getNumberStored()>0)
	{
		int i =  contextList->currentRow();
		storedContext.switchCurrentEnvTo(i);
	}
	else{
		std::cout << "Warning: no context in stack" << std::endl;
	}
}

void qtTestWindow::resetContext()
{
	storedContext.clear();
//	setContextUserApp(context);
	for(uint i=0;i<itemList.size();i++)
	{
		delete itemList.at(i);
	}
	itemList.clear();
}

void qtTestWindow::runAllRRT()
{
//	runAllRounds->setDisabled(true);
	std::string str = "MultiRRT";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void qtTestWindow::runAllGreedy()
{
//	runAllRounds->setDisabled(true);
	std::string str = "MultiGreedy";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void qtTestWindow::showHistoWindow()
{
#ifdef QWT
	histoWin = new HistoWindow();
	histoWin->startWindow();
#endif
}


qtTestWindow::~qtTestWindow()
{
	//delete
//	delete box;
//	delete Layout;
}

#include "moc_qtTestWindow.cpp"
