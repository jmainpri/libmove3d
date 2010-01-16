#include "mainwindow.hpp"
#include "ui_mainwindow.hpp"
#include "../cppToQt.hpp"
//#include "qdebugstream.hpp"
#include "../qtOpenGL/glwidget.hpp"
#include "Graphic-pkg.h"
#include "../qtBase/SpinBoxSliderConnector_p.hpp"

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

    QPalette pal(ui->centralWidget->palette()); // copy widget's palette to non const QPalette
    QColor myColor(Qt::darkGray);
    pal.setColor(QPalette::Window,myColor);
    ui->centralWidget->setPalette( pal );        // set the widget's palette

    cout << "pipe2openGl = new Move3D2OpenGl(ui->OpenGL)" << endl;

//    connect(ui->actionOpen,SIGNAL(triggered()),this,SLOT(open()));
    connect(ui->actionOpenScenario,SIGNAL(triggered()),this,SLOT(openScenario()));

    connect(ui->actionKCDPropietes,SIGNAL(triggered()),mKCDpropertiesWindow,SLOT(show()));

    //    connect(ui->pagesOfStakedWidget, SIGNAL(activated(int)),ui->stackedWidget, SLOT(setCurrentIndex(int)));

    connectCheckBoxes();

    connect(ui->pushButtonRun,SIGNAL(clicked(bool)),this,SLOT(run()),Qt::DirectConnection);

    connect(ui->pushButtonReset,SIGNAL(clicked(bool)),this,SLOT(reset()),Qt::DirectConnection);
    connect(ui->pushButtonStop,SIGNAL(clicked(bool)),this,SLOT(stop()),Qt::DirectConnection);

    ui->pushButtonRun->setDisabled(false);
    ui->pushButtonStop->setDisabled(true);
    ui->pushButtonReset->setDisabled(true);

    connect(ENV.getObject(Env::isPRMvsDiffusion), SIGNAL(valueChanged(bool)), ui->radioButtonPRM, SLOT(setChecked(bool)), Qt::DirectConnection);
    connect(ui->radioButtonPRM, SIGNAL(toggled(bool)), ENV.getObject(Env::isPRMvsDiffusion), SLOT(set(bool)), Qt::DirectConnection);
    ui->radioButtonDiff->setChecked(!ENV.getBool(Env::isPRMvsDiffusion));

    connectCheckBoxToEnv(ui->checkBoxWithShortCut,      Env::withShortCut);

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

    connect(ui->pushButtonResetGraph,SIGNAL(clicked()),this,SLOT(ResetGraph()));

    connectCheckBoxToEnv(ui->checkBoxDrawLightSource,      Env::drawLightSource);

    vector<double>  envSize(6);
    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;

    ui->doubleSpinBoxLightX->setMinimum(2*envSize[0]);
    ui->doubleSpinBoxLightX->setMaximum(2*envSize[1]);
    ui->doubleSpinBoxLightY->setMinimum(2*envSize[2]);
    ui->doubleSpinBoxLightY->setMaximum(2*envSize[3]);
    ui->doubleSpinBoxLightZ->setMinimum(2*envSize[4]);
    ui->doubleSpinBoxLightZ->setMaximum(2*envSize[5]);

    QtShiva::SpinBoxSliderConnector *connectorLightX = new QtShiva::SpinBoxSliderConnector(
            this, ui->doubleSpinBoxLightX, ui->horizontalSliderLightX);
    QtShiva::SpinBoxSliderConnector *connectorLightY = new QtShiva::SpinBoxSliderConnector(
            this, ui->doubleSpinBoxLightY, ui->horizontalSliderLightY);
    QtShiva::SpinBoxSliderConnector *connectorLightZ = new QtShiva::SpinBoxSliderConnector(
            this, ui->doubleSpinBoxLightZ, ui->horizontalSliderLightZ);

    connect(connectorLightX,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosX()));
    connect(connectorLightY,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosY()));
    connect(connectorLightZ,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosZ()));

    connect(connectorLightX,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));
    connect(connectorLightY,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));
    connect(connectorLightZ,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));

    ui->doubleSpinBoxLightX->setValue(G3D_WIN->lightPosition[0]);
    ui->doubleSpinBoxLightY->setValue(G3D_WIN->lightPosition[1]);
    ui->doubleSpinBoxLightZ->setValue(G3D_WIN->lightPosition[2]);
}

MainWindow::~MainWindow()
{
    delete mKCDpropertiesWindow;
#ifdef QWT
    delete ui->sidePannel->plot;
#endif
    delete ui;
}

void MainWindow::openScenario()
{
    QString fileName = QFileDialog::getOpenFileName(this);
    //     if (!fileName.isEmpty())
    //         loadFile(fileName);

    cout << "Open scenarion " << fileName.toStdString() << endl;
}

void MainWindow::connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p)
{
    connect(ENV.getObject(p), SIGNAL(valueChanged(bool)), box, SLOT(setChecked(bool)), Qt::DirectConnection);
    connect(box, SIGNAL(toggled(bool)), ENV.getObject(p), SLOT(set(bool)), Qt::DirectConnection);
    box->setChecked(ENV.getBool(p));
}

void MainWindow::changeLightPosX()
{
    float* lightPosition = G3D_WIN->lightPosition;
    lightPosition[0] = ui->doubleSpinBoxLightX->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN);
//    cout << "Change X value" << endl;
}

void MainWindow::changeLightPosY()
{
    float* lightPosition = G3D_WIN->lightPosition;
    lightPosition[1] = ui->doubleSpinBoxLightY->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN);
//    cout << "Change Y value" << endl;
}

void MainWindow::changeLightPosZ()
{
    float* lightPosition = G3D_WIN->lightPosition;
    lightPosition[2] = ui->doubleSpinBoxLightZ->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN);
//    cout << "Change Z value" << endl;
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

    connect(ui->checkBoxSmooth, SIGNAL(toggled(bool)), this , SLOT(setBoolSmooth(bool)), Qt::DirectConnection);
    connect(ui->checkBoxSmooth, SIGNAL(toggled(bool)), ui->OpenGL , SLOT(updateGL()));

    connect(ui->checkBoxFilaire, SIGNAL(toggled(bool)), this , SLOT(setBoolFilaire(bool)), Qt::DirectConnection);
    connect(ui->checkBoxFilaire, SIGNAL(toggled(bool)), ui->OpenGL , SLOT(updateGL()));

    connectCheckBoxToEnv(ui->checkBoxAxis, Env::drawFrame);
    connect(ui->checkBoxAxis, SIGNAL(toggled(bool)), ui->OpenGL , SLOT(updateGL()));
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

void MainWindow::setBoolSmooth(bool value)
{
    G3D_WIN->GOURAUD = value;
}

void MainWindow::setBoolFilaire(bool value)
{
    G3D_WIN->FILAIRE = value;
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
    cout << "Run Motion Planning" << endl;

    if(!ENV.getBool(Env::isPRMvsDiffusion))
    {
        std::string str = "RunDiffusion";
        write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    }
    else
    {
        std::string str = "RunPRM";
        write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    }

    ENV.setBool(Env::isRunning,true);
}

void MainWindow::stop()
{
    p3d_SetStopValue(true);
    ENV.setBool(Env::isRunning,false);
}

void MainWindow::reset()
{
    std::string str = "ResetGraph";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    ui->pushButtonRun->setDisabled(false);
    ui->pushButtonStop->setDisabled(true);
    ui->pushButtonReset->setDisabled(true);
    p3d_SetStopValue(false);
}

void MainWindow::isPlanning()
{
    ui->pushButtonRun->setDisabled(true);
    ui->pushButtonReset->setDisabled(true);
    ui->pushButtonStop->setDisabled(false);

    ENV.setBool(Env::isRunning,true);

    QPalette pal(Qt::lightGray); // copy widget's palette to non const QPalette
    ui->toolBox->setPalette( pal );        // set the widget's palette
}

void MainWindow::planningFinished()
{
    if( ENV.getBool(Env::isRunning) == false )
    {
        ui->pushButtonStop->setDisabled(true);
        ui->pushButtonReset->setDisabled(false);

        QPalette pal(Qt::white); // copy widget's palette to non const QPalette
        ui->toolBox->setPalette( pal );        // set the widget's palette
    }
    else
    {
        this->isPlanning();
    }
}
void MainWindow::drawAllWinActive()
{
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::ResetGraph()
{
    std::string str = "ResetGraph";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

// Show Traj functions ------------------------------------------
// --------------------------------------------------------------
static int traj_play = TRUE;
GLWidget* ptrOpenGL;

static int default_drawtraj_fct_qt_pipe(p3d_rob* robot, p3d_localpath* curLp)
{
      std::string str = "g3d_draw_allwin_active";
      write(qt_fl_pipe[1],str.c_str(),str.length()+1);
//    ptrOpenGL->updateGL();

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
