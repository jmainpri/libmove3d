#include "qtTestWindow.hpp"

#include <iostream>
#include <string>

#include "p3d/env.hpp"

qtTestWindow::qtTestWindow() : qtBaseWindow() {
	init();
	_Context = new SaveContext();
}

// Constructor
void qtTestWindow::init()
{
	string = tr("Test Context");

//	LabeledSlider* nbTest = createSlider(tr("Nb Of rounds"), Env::nb_rounds, 0, 300);
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

	_Context->saveCurrentEnvToStack();
}

void qtTestWindow::printAllContext()
{
	if(_Context->getNumberStored()>0){

		for(uint i=0;i<_Context->getNumberStored();i++){
			std::cout << "------------ Context Number " << i << " ------------" << std::endl;
			_Context->printData(i);
		}
		std::cout << "-------------------------------------------" << std::endl;
		std::cout << " Total number of contexts in stack =  " << _Context->getNumberStored() << std::endl;
		std::cout << "-------------------------------------------" << std::endl;
	}
	else{
		std::cout << "Warning: no context in stack" << std::endl;
	}
}

void qtTestWindow::printContext()
{
	if(_Context->getNumberStored()>0){
		int i =  contextList->currentRow();
		std::cout << "------------ Context Number " << i << " ------------" << std::endl;
		_Context->printData(i);
	}
	else{
		std::cout << "Warning: no context in stack" << std::endl;
	}
}

void qtTestWindow::setToSelected()
{
	if(_Context->getNumberStored()>0){
		int i =  contextList->currentRow();
		_Context->switchCurrentEnvTo(i);
	}
	else{
		std::cout << "Warning: no context in stack" << std::endl;
	}
}

void qtTestWindow::resetContext()
{
	_Context->clear();
//	setContextUserApp(context);
	for(uint i=0;i<itemList.size();i++)
	{
		delete itemList.at(i);
	}
	itemList.clear();
}




qtTestWindow::~qtTestWindow()
{
	//delete
//	delete box;
//	delete Layout;
}

#include "moc_qtTestWindow.cpp"
