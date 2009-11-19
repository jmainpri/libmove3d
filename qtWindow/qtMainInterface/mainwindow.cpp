#include "mainwindow.hpp"
#include "ui_mainwindow.hpp"
#include "../cppToQt.hpp"
//#include "qdebugstream.hpp"
#include "../qtOpenGL/glwidget.hpp"
#include "Graphic-pkg.h"

Move3D2OpenGl* pipe2openGl;

using namespace std;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->sidePannel->setMainWindow(this);

    mKCDpropertiesWindow = new KCDpropertiesWindow();

    ui->OpenGL->setWinSize(G3D_WIN->size);
    pipe2openGl = new Move3D2OpenGl(ui->OpenGL);

    cout << "pipe2openGl = new Move3D2OpenGl(ui->OpenGL)" << endl;

    connect(ui->actionOpen,SIGNAL(triggered()),this,SLOT(open()));
    connect(ui->actionOpenScenario,SIGNAL(triggered()),this,SLOT(open()));

    connect(ui->actionKCDPropietes,SIGNAL(triggered()),mKCDpropertiesWindow,SLOT(show()));

//    connect(ui->pagesOfStakedWidget, SIGNAL(activated(int)),ui->stackedWidget, SLOT(setCurrentIndex(int)));

    connectCheckBoxes();

    connect(ui->pushButtonRun,SIGNAL(clicked(bool)),this,SLOT(isPlanning()),Qt::DirectConnection);
    connect(ui->pushButtonRun,SIGNAL(clicked(bool)),this,SLOT(run()),Qt::DirectConnection);

    connect(ui->pushButtonReset,SIGNAL(clicked(bool)),this,SLOT(reset()),Qt::DirectConnection);
    connect(ui->pushButtonStop,SIGNAL(clicked(bool)),this,SLOT(stop()),Qt::DirectConnection);

    ui->pushButtonRun->setDisabled(false);
    ui->pushButtonStop->setDisabled(true);
    ui->pushButtonReset->setDisabled(true);

    connectCheckBoxToEnv(ui->checkBoxIsRunning,Env::isRunning);

    connect( ENV.getObject(Env::isRunning), SIGNAL(valueChanged(bool)), this, SLOT(planningFinished(void)), Qt::DirectConnection);

    connect(ui->checkBoxDrawGraph,SIGNAL(toggled(bool)),this,SLOT(drawAllWinActive()),Qt::DirectConnection);
    connect(ui->checkBoxDrawTraj,SIGNAL(toggled(bool)),this,SLOT(drawAllWinActive()),Qt::DirectConnection);

    connectCheckBoxToEnv(ui->checkBoxDrawGraph,Env::drawGraph);
    connectCheckBoxToEnv(ui->checkBoxDrawTraj,Env::drawTraj);
    ui->checkBoxDrawGraph->setCheckState(Qt::Checked);

    connect(ui->pushButtonShowTraj,SIGNAL(clicked(bool)),this,SLOT(showTraj()),Qt::DirectConnection);

//    ui->consoleOutput->setTextFormat(Qt::LogText);
//    QDebugStream qout(cout, ui->consoleOutput);

    ui->progressBar->setValue(0);
//    cout << "Max progress bar" << ui->progressBar->maximum() << endl;
    connect(ENV.getObject(Env::progress),SIGNAL(valueChanged(int)),ui->progressBar,SLOT(setValue(int)));

    connect(ui->pushButtonRestoreView,SIGNAL(clicked(bool)),this,SLOT(restoreView()),Qt::DirectConnection);

}

MainWindow::~MainWindow()
{
    delete mKCDpropertiesWindow;
#ifdef QWT
    delete ui->sidePannel->plot;
#endif
    delete ui;
}

 void MainWindow::open()
 {
     QString fileName = QFileDialog::getOpenFileName(this);
//     if (!fileName.isEmpty())
//         loadFile(fileName);
 }

 void MainWindow::connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p)
{
    connect(ENV.getObject(p), SIGNAL(valueChanged(bool)), box, SLOT(setChecked(bool)), Qt::DirectConnection);
    connect(box, SIGNAL(toggled(bool)), ENV.getObject(p), SLOT(set(bool)), Qt::DirectConnection);
    box->setChecked(ENV.getBool(p));
}

 // Viewer Buttons -----------------------------------------------
// ---------------------------------------------------------------
 void MainWindow::connectCheckBoxes()
{
        connect(ui->checkBoxGhosts, SIGNAL(toggled(bool)), this , SLOT(setBoolGhost(bool)), Qt::DirectConnection);
        connect(ui->checkBoxGhosts, SIGNAL(toggled(bool)), ui->OpenGL , SLOT(updateGL()));

        connect(ui->checkBoxBB, SIGNAL(toggled(bool)), this , SLOT(setBoolBb(bool)), Qt::DirectConnection);
        connect(ui->checkBoxBB, SIGNAL(toggled(bool)), ui->OpenGL , SLOT(updateGL()));

        connect(ui->checkBoxFloor, SIGNAL(toggled(bool)), this , SLOT(setBoolFloor(bool)), Qt::DirectConnection);
        connect(ui->checkBoxFloor, SIGNAL(toggled(bool)), ui->OpenGL , SLOT(updateGL()));
        ui->checkBoxFloor->setCheckState(Qt::Checked);

        connect(ui->checkBoxTiles, SIGNAL(toggled(bool)), this , SLOT(setBoolTiles(bool)), Qt::DirectConnection);
        connect(ui->checkBoxTiles, SIGNAL(toggled(bool)), ui->OpenGL , SLOT(updateGL()));
        ui->checkBoxTiles->setCheckState(Qt::Checked);

        connect(ui->checkBoxWalls, SIGNAL(toggled(bool)), this , SLOT(setBoolWalls(bool)), Qt::DirectConnection);
        connect(ui->checkBoxWalls, SIGNAL(toggled(bool)), ui->OpenGL , SLOT(updateGL()));

        connect(ui->checkBoxShadows, SIGNAL(toggled(bool)), this , SLOT(setBoolShadows(bool)), Qt::DirectConnection);
        connect(ui->checkBoxShadows, SIGNAL(toggled(bool)), ui->OpenGL , SLOT(updateGL()));
}

void MainWindow::setBoolGhost(bool value)
{
        G3D_WIN->GHOST = value;
}

void MainWindow::setBoolBb(bool value)
{
        G3D_WIN->BB = value;
}


void MainWindow::setBoolFloor(bool value)
{
        G3D_WIN->displayFloor = value;
}


void MainWindow::setBoolTiles(bool value)
{
        G3D_WIN->displayTiles = value;
}


void MainWindow::setBoolWalls(bool value)
{
        G3D_WIN->displayWalls = value;
}

void MainWindow::setBoolShadows(bool value)
{
        G3D_WIN->displayShadows = value;
}

void MainWindow::restoreView()
{
    g3d_restore_win_camera(G3D_WIN);
    drawAllWinActive();
}

// Run Buttons -----------------------------------------------
// -----------------------------------------------------------
void MainWindow::run()
{
    cout << "Run" << endl;
    cout << "ENV.getBool(Env::Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;
    std::string str = "p3d_RunDiffusion";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::stop()
{
     p3d_SetStopValue(true);
     ENV.setBool(Env::isRunning,false);
}

void MainWindow::reset()
{
    p3d_del_graph(XYZ_GRAPH);
    std::string str = "ResetGraph";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    ui->pushButtonRun->setDisabled(false);
    ui->pushButtonStop->setDisabled(true);
    ui->pushButtonReset->setDisabled(true);
}

void MainWindow::isPlanning()
{
    ENV.setBool(Env::isRunning,true);
    ui->pushButtonRun->setDisabled(true);
    ui->pushButtonReset->setDisabled(true);
    ui->pushButtonStop->setDisabled(false);
}

void MainWindow::planningFinished()
{
    if(ENV.getBool(Env::isRunning) == false )
    {
        ui->pushButtonStop->setDisabled(true);
        ui->pushButtonReset->setDisabled(false);
    }
}
void MainWindow::drawAllWinActive()
{
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

// Show Traj functions ------------------------------------------
// --------------------------------------------------------------
static int traj_play = TRUE;
GLWidget* ptrOpenGL;

static int default_drawtraj_fct_qt_pipe(p3d_rob* robot, p3d_localpath* curLp)
{
//  std::string str = "g3d_draw_allwin_active";
//  write(qt_fl_pipe[1],str.c_str(),str.length()+1);
  ptrOpenGL->updateGL();

  return(traj_play);
}

void MainWindow::showTraj()
{
    cout << "Show Traj....."<< endl;
    ptrOpenGL = ui->OpenGL;
    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    g3d_show_tcur_rob(robotPt,default_drawtraj_fct_qt_pipe);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
//    cout << "Key pressed" << endl;
    switch(event->key())
    {
         case Qt::Key_X:
                mouse_mode = 1;
                cout << "Switch to second" << endl;
                break;

         case Qt::Key_C:
                mouse_mode = 2;
                cout << "Switch to third" << endl;
                break;

//            if(mouse_mode == 2)
//            {
//                mouse_mode = 0;
//                cout << "Switch to normal" << endl;
//                return;
//            }
//                break;
     }
}

void MainWindow::keyReleaseEvent(QKeyEvent *e)
{
     mouse_mode = 0;
}

#include "moc_mainwindow.cpp"
