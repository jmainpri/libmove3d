#include "qtMainWindow.hpp"
#include "cppToQt.hpp"

#include "qtMyWindows/qtHriWindow.hpp"
#include "qtMyWindows/qtDiffusionWindow.hpp"
#include "qtMyWindows/qtTestWindow.hpp"
#include "qtMyWindows/qtVisuWindow.hpp"
#include "qtMyWindows/qtGreedyWindow.hpp"
#include "qtMyWindows/qtOptimWindow.hpp"
#include "qtMyWindows/qtColisionTestWindow.hpp"
#include "qtMyWindows/qtESTWindow.hpp"

#include "../planning_api/planningAPI.hpp"

using namespace std;

MainWidget::MainWidget(QWidget* parent) :

    QWidget(parent) {

	// Define Basic Structure ---------------
	// --------------------------------------

    //setFont(QFont(font().defaultFamily(), 12));
    layout = new QBoxLayout(QBoxLayout::TopToBottom);
    master_layout = new QBoxLayout(QBoxLayout::LeftToRight);
    left_layout = new QBoxLayout(QBoxLayout::TopToBottom);

    layout->setSpacing(3);

    stackedWidget = new QStackedWidget;

    pageList = new QListWidget;
    pageList->setSelectionMode(QAbstractItemView::SingleSelection);
    connect(pageList, SIGNAL(currentRowChanged(int)),stackedWidget, SLOT(setCurrentIndex(int)));
    pageList->setCurrentRow(0);

    /*for(unsigned int i = 0; i < style_db.styles.size(); i++) {
      styles->insertItem(i, style_db.styles[i]);
    }

    connect(styles, SIGNAL(activated(int)), &style_db, SLOT(set_style(int)));
    connect(p, SIGNAL(clicked()), l, SLOT(showHide()));*/

    // Right Layout------------------------
    // ------------------------------------

    vector<qtBaseWindow*> Frames;

    // Add your Frame to the vector
    Frames.push_back(new qtDiffusionWindow());
    Frames.push_back(new qtHriWindow());
    Frames.push_back(new qtTestWindow());
    Frames.push_back(new qtVisuWindow());
    Frames.push_back(new qtGreedyWindow());
    Frames.push_back(new qtOptimWindow());
    Frames.push_back(new qtColisionTestWindow());
    Frames.push_back(new qtESTWindow());

    for(uint i=0;i<Frames.size();i++){
    	addWindow(Frames.at(i));
    }

    pageList->setMaximumWidth(150);
    left_layout->addWidget(pageList, 0, Qt::AlignTop);

    layout->addWidget(stackedWidget);

    // Left Layout-------------------------
    // ------------------------------------

    // Basic Buttons-----------------------

    left_layout->addWidget(pageList, 0, Qt::AlignBottom);

    runButton = new QPushButton(tr("Run"));
    stopButton = new QPushButton(tr("Stop"));
    resetButton = new QPushButton(tr("Reset"));
    miscButton = new QPushButton(tr("Misc"));

    connect(runButton, SIGNAL(clicked()), this, SLOT(run()),Qt::DirectConnection);
    connect(resetButton, SIGNAL(clicked()), this, SLOT(reset()),Qt::DirectConnection);
    connect(stopButton, SIGNAL(clicked()), this, SLOT(stop()),Qt::DirectConnection);
    connect(miscButton, SIGNAL(clicked()), this, SLOT(misc()),Qt::DirectConnection);

    left_layout->addWidget(runButton);
    left_layout->setAlignment(runButton, Qt::AlignLeft);

    left_layout->addWidget(stopButton);
    left_layout->setAlignment(stopButton, Qt::AlignLeft);

    left_layout->addWidget(resetButton);
    left_layout->setAlignment(resetButton, Qt::AlignLeft);

    left_layout->addWidget(miscButton);
    left_layout->setAlignment(miscButton, Qt::AlignLeft);


    // Check boxes -------------------------

    QCheckBox* drawGraphCheckBox = createCheckBox(tr("Draw Graph"), Env::drawGraph);
    QCheckBox* drawTrajCheckBox = createCheckBox(tr("Draw Traj"), Env::drawTraj);
    QCheckBox* isCostSpace = createCheckBox(tr("Cost Space"), Env::isCostSpace);

    connect(ENV.getObject(Env::drawTraj), SIGNAL(valueChanged(bool)), this, SLOT(drawTraj()));
    connect(ENV.getObject(Env::drawGraph), SIGNAL(valueChanged(bool)), this, SLOT(drawGraph()));

    left_layout->addWidget(drawGraphCheckBox);
    left_layout->setAlignment(drawGraphCheckBox, Qt::AlignLeft);
    left_layout->addWidget(drawTrajCheckBox);
    left_layout->setAlignment(drawTrajCheckBox, Qt::AlignLeft);
    left_layout->addWidget(isCostSpace);
    left_layout->setAlignment(isCostSpace, Qt::AlignLeft);
//
//    left_layout->addWidget(drawTrajCheckBox);
//    left_layout->setAlignment(drawTrajCheckBox, Qt::AlignLeft);

    //left_layout->setSpacing(3);
    //left_layout->margin(2);

    // Add All to Master Layout------------
    // ------------------------------------
    master_layout->addLayout(left_layout, 0);
    master_layout->addLayout(layout, 1);

    layout->addStretch();

    setLayout(master_layout);
  }


QCheckBox* MainWidget::createCheckBox(QString s, Env::boolParameter p)
  {
    QCheckBox* box = new QCheckBox(s);
    connect(ENV.getObject(p), SIGNAL(valueChanged(bool)), box, SLOT(setChecked(bool)), Qt::DirectConnection);
    connect(box, SIGNAL(toggled(bool)), ENV.getObject(p), SLOT(set(bool)), Qt::DirectConnection);
    box->setChecked(ENV.getBool(p));
    return(box);
  }

void MainWidget::addWindow(qtBaseWindow* Frame)
{
	stackedWidget->addWidget(Frame->getBox());
	pageList->addItem(Frame->getString());
}


  void MainWidget::waitThreads() {
    exit(0);
  }

  void MainWidget::drawGraph()
  {
	cout << "change drawGraph" << endl;

    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
  }
  void MainWidget::drawTraj()
  {
  	cout << "change drawTraj" << endl;

  	std::string str = "g3d_draw_allwin_active";
  	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
  }

  void MainWidget::run()
  {
//	  stop();
//	  reset();

	  runButton->setDisabled(true);
	  resetButton->setDisabled(true);

	  std::string str = "p3d_RunDiffusion";
	  write(qt_fl_pipe[1],str.c_str(),str.length()+1);
  }

  void MainWidget::misc()
  {
	  runButton->setDisabled(true);
	  resetButton->setDisabled(true);
	  std::string str = "p3d_RunGreedy";
	  write(qt_fl_pipe[1],str.c_str(),str.length()+1);
  }

  void MainWidget::stop()
  {
	  p3d_SetStopValue(true);
	  resetButton->setDisabled(false);
  }

  void MainWidget::reset()
  {
    p3d_del_graph(XYZ_GRAPH);
    std::string str = "ResetGraph";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    runButton->setDisabled(false);
  }

  MainWidget::~MainWidget()
  {
  }

#include "moc_qtMainWindow.cpp"
