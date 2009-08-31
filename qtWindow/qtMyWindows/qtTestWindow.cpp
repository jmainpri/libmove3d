#include "qtTestWindow.hpp"
#include "UserAppli-pkg.h"
#include <iostream>
#include <string>


qtTestWindow::qtTestWindow() : qtBaseWindow() {
	init();
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

//	Layout->addWidget(nbTest);
	Layout->addWidget(StringLabel);
	Layout->addWidget(nameEdit);
	Layout->addWidget(saveContext);
	Layout->addWidget(resetContext);
	Layout->addWidget(printAllContext);
	Layout->addWidget(contextList);
	Layout->addWidget(printSelectedContext);
	Layout->addWidget(setSelectedContext);
//	Layout->addWidget(multCost);

	// Set Spacer
	QWidget* spacer = new QWidget();
	spacer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);
	Layout->addWidget(spacer);

}

void qtTestWindow::saveContext()
{
//	ENV.setString(Env::nameOfFile,nameEdit->text().toStdString());
//	testContext newContext;
//	newContext.saveContext();
//	context.push_back(newContext);
//	setContextUserApp(context);
	QListWidgetItem* item= new QListWidgetItem(contextList);
	itemList.push_back(item);
	itemList.back()->setText(nameEdit->text());
}

void qtTestWindow::printAllContext()
{
//	if(context.size()>0){
//
//		for(uint i=0;i<context.size();i++){
//			std::cout << "------------ Context Number " << i << " ------------" << std::endl;
//			context.at(i).print();
//		}
//		std::cout << "-------------------------------------------" << std::endl;
//		std::cout << " Total number of contexts in stack =  " << context.size() << std::endl;
//		std::cout << "-------------------------------------------" << std::endl;
//	}
//	else{
//		std::cout << "Warning: no context in stack" << std::endl;
//	}
}

void qtTestWindow::printContext()
{
//	if(context.size()>0){
//		int i =  contextList->currentRow();
//		std::cout << "------------ Context Number " << i << " ------------" << std::endl;
//		context.at(i).print();
//	}
//	else{
//		std::cout << "Warning: no context in stack" << std::endl;
//	}
}

void qtTestWindow::setToSelected()
{
//	if(context.size()>0){
//		int i =  contextList->currentRow();
//		context.at(i).setContext();
//	}
//	else{
//		std::cout << "Warning: no context in stack" << std::endl;
//	}
}

void qtTestWindow::resetContext()
{
//	context.clear();
//	setContextUserApp(context);
//	for(uint i=0;i<itemList.size();i++)
//	{
//		delete itemList.at(i);
//	}
//	itemList.clear();
}




qtTestWindow::~qtTestWindow()
{
	//delete
//	delete box;
//	delete Layout;
}

#include "moc_qtTestWindow.cpp"
