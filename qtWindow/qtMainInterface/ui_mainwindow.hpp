/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Fri Dec 11 14:04:44 2009
**      by: Qt User Interface Compiler version 4.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QScrollBar>
#include <QtGui/QSlider>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "../qtOpenGL/glwidget.hpp"
#include "./sidewindow.hpp"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionQuit;
    QAction *actionRobot;
    QAction *action3DViewer;
    QAction *actionKCDPropietes;
    QAction *actionAbout;
    QAction *actionCloseEnvironement;
    QAction *actionOpenScenario;
    QAction *actionOpen;
    QAction *actionSaveScenarion;
    QAction *actionNameOfEnv;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout_2;
    SideWindow *sidePannel;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout_3;
    GLWidget *OpenGL;
    QProgressBar *progressBar;
    QToolBox *toolBox;
    QWidget *RunMotion;
    QWidget *RunMotionPlan;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *allButtonsLayout;
    QPushButton *pushButtonRun;
    QPushButton *pushButtonStop;
    QPushButton *pushButtonReset;
    QCheckBox *checkBoxWithShortCut;
    QRadioButton *radioButtonDiff;
    QRadioButton *radioButtonPRM;
    QPushButton *pushButtonResetGraph;
    QWidget *ViewerSettings;
    QGroupBox *groupBoxScene;
    QCheckBox *checkBoxTiles;
    QCheckBox *checkBoxWalls;
    QCheckBox *checkBoxFloor;
    QCheckBox *checkBoxShadows;
    QCheckBox *checkBoxSmooth;
    QCheckBox *checkBoxAxis;
    QGroupBox *groupBoxMPlanning;
    QCheckBox *checkBoxDrawGraph;
    QCheckBox *checkBoxDrawTraj;
    QGroupBox *groupBox3DModel;
    QCheckBox *checkBoxBB;
    QCheckBox *checkBoxGhosts;
    QCheckBox *checkBoxFilaire;
    QWidget *Viewersettings2;
    QDoubleSpinBox *doubleSpinBoxLightZ;
    QDoubleSpinBox *doubleSpinBoxLightX;
    QDoubleSpinBox *doubleSpinBoxLightY;
    QPushButton *pushButtonRestoreView;
    QSlider *horizontalSliderLightX;
    QSlider *horizontalSliderLightY;
    QSlider *horizontalSliderLightZ;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QPushButton *pushButtonRestoreLight;
    QCheckBox *checkBoxDrawLightSource;
    QWidget *page;
    QPushButton *pushButtonShowTraj;
    QLineEdit *lineEdit;
    QScrollBar *horizontalScrollBar;
    QLabel *label;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuHelp;
    QMenu *menuCollisionCheker;
    QMenu *menuTrajectory;
    QMenu *menuEnvironement_2;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->setWindowModality(Qt::NonModal);
        MainWindow->setEnabled(true);
        MainWindow->resize(1319, 704);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setSizeIncrement(QSize(0, 0));
        actionQuit = new QAction(MainWindow);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
        actionRobot = new QAction(MainWindow);
        actionRobot->setObjectName(QString::fromUtf8("actionRobot"));
        action3DViewer = new QAction(MainWindow);
        action3DViewer->setObjectName(QString::fromUtf8("action3DViewer"));
        actionKCDPropietes = new QAction(MainWindow);
        actionKCDPropietes->setObjectName(QString::fromUtf8("actionKCDPropietes"));
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionCloseEnvironement = new QAction(MainWindow);
        actionCloseEnvironement->setObjectName(QString::fromUtf8("actionCloseEnvironement"));
        actionOpenScenario = new QAction(MainWindow);
        actionOpenScenario->setObjectName(QString::fromUtf8("actionOpenScenario"));
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        actionSaveScenarion = new QAction(MainWindow);
        actionSaveScenarion->setObjectName(QString::fromUtf8("actionSaveScenarion"));
        actionNameOfEnv = new QAction(MainWindow);
        actionNameOfEnv->setObjectName(QString::fromUtf8("actionNameOfEnv"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        sizePolicy.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy);
        horizontalLayout_2 = new QHBoxLayout(centralWidget);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setSizeConstraint(QLayout::SetNoConstraint);
        horizontalLayout_2->setContentsMargins(-1, -1, -1, 0);
        sidePannel = new SideWindow(centralWidget);
        sidePannel->setObjectName(QString::fromUtf8("sidePannel"));
        sidePannel->setMinimumSize(QSize(500, 0));

        horizontalLayout_2->addWidget(sidePannel);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMinimumSize(QSize(400, 300));
        horizontalLayout_3 = new QHBoxLayout(groupBox);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        OpenGL = new GLWidget(groupBox);
        OpenGL->setObjectName(QString::fromUtf8("OpenGL"));
        sizePolicy.setHeightForWidth(OpenGL->sizePolicy().hasHeightForWidth());
        OpenGL->setSizePolicy(sizePolicy);
        OpenGL->setMinimumSize(QSize(400, 300));

        horizontalLayout_3->addWidget(OpenGL);


        verticalLayout->addWidget(groupBox);

        progressBar = new QProgressBar(centralWidget);
        progressBar->setObjectName(QString::fromUtf8("progressBar"));
        progressBar->setValue(24);

        verticalLayout->addWidget(progressBar);

        toolBox = new QToolBox(centralWidget);
        toolBox->setObjectName(QString::fromUtf8("toolBox"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(toolBox->sizePolicy().hasHeightForWidth());
        toolBox->setSizePolicy(sizePolicy1);
        toolBox->setMinimumSize(QSize(0, 250));
        RunMotion = new QWidget();
        RunMotion->setObjectName(QString::fromUtf8("RunMotion"));
        RunMotion->setGeometry(QRect(0, 0, 96, 26));
        RunMotionPlan = new QWidget(RunMotion);
        RunMotionPlan->setObjectName(QString::fromUtf8("RunMotionPlan"));
        RunMotionPlan->setGeometry(QRect(10, 0, 151, 91));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(RunMotionPlan->sizePolicy().hasHeightForWidth());
        RunMotionPlan->setSizePolicy(sizePolicy2);
        RunMotionPlan->setStyleSheet(QString::fromUtf8(""));
        horizontalLayout_4 = new QHBoxLayout(RunMotionPlan);
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        allButtonsLayout = new QVBoxLayout();
        allButtonsLayout->setSpacing(6);
        allButtonsLayout->setObjectName(QString::fromUtf8("allButtonsLayout"));
        pushButtonRun = new QPushButton(RunMotionPlan);
        pushButtonRun->setObjectName(QString::fromUtf8("pushButtonRun"));

        allButtonsLayout->addWidget(pushButtonRun);

        pushButtonStop = new QPushButton(RunMotionPlan);
        pushButtonStop->setObjectName(QString::fromUtf8("pushButtonStop"));

        allButtonsLayout->addWidget(pushButtonStop);

        pushButtonReset = new QPushButton(RunMotionPlan);
        pushButtonReset->setObjectName(QString::fromUtf8("pushButtonReset"));

        allButtonsLayout->addWidget(pushButtonReset);


        horizontalLayout_4->addLayout(allButtonsLayout);

        checkBoxWithShortCut = new QCheckBox(RunMotion);
        checkBoxWithShortCut->setObjectName(QString::fromUtf8("checkBoxWithShortCut"));
        checkBoxWithShortCut->setGeometry(QRect(20, 120, 121, 21));
        radioButtonDiff = new QRadioButton(RunMotion);
        radioButtonDiff->setObjectName(QString::fromUtf8("radioButtonDiff"));
        radioButtonDiff->setGeometry(QRect(180, 10, 91, 31));
        radioButtonPRM = new QRadioButton(RunMotion);
        radioButtonPRM->setObjectName(QString::fromUtf8("radioButtonPRM"));
        radioButtonPRM->setGeometry(QRect(180, 50, 61, 31));
        pushButtonResetGraph = new QPushButton(RunMotion);
        pushButtonResetGraph->setObjectName(QString::fromUtf8("pushButtonResetGraph"));
        pushButtonResetGraph->setGeometry(QRect(600, 40, 91, 20));
        toolBox->addItem(RunMotion, QString::fromUtf8("Run Motion Planning"));
        ViewerSettings = new QWidget();
        ViewerSettings->setObjectName(QString::fromUtf8("ViewerSettings"));
        ViewerSettings->setGeometry(QRect(0, 0, 96, 26));
        groupBoxScene = new QGroupBox(ViewerSettings);
        groupBoxScene->setObjectName(QString::fromUtf8("groupBoxScene"));
        groupBoxScene->setGeometry(QRect(10, 0, 261, 91));
        checkBoxTiles = new QCheckBox(groupBoxScene);
        checkBoxTiles->setObjectName(QString::fromUtf8("checkBoxTiles"));
        checkBoxTiles->setGeometry(QRect(10, 30, 71, 21));
        checkBoxWalls = new QCheckBox(groupBoxScene);
        checkBoxWalls->setObjectName(QString::fromUtf8("checkBoxWalls"));
        checkBoxWalls->setGeometry(QRect(90, 30, 71, 21));
        checkBoxFloor = new QCheckBox(groupBoxScene);
        checkBoxFloor->setObjectName(QString::fromUtf8("checkBoxFloor"));
        checkBoxFloor->setGeometry(QRect(10, 60, 61, 21));
        checkBoxShadows = new QCheckBox(groupBoxScene);
        checkBoxShadows->setObjectName(QString::fromUtf8("checkBoxShadows"));
        checkBoxShadows->setGeometry(QRect(90, 60, 81, 21));
        checkBoxSmooth = new QCheckBox(groupBoxScene);
        checkBoxSmooth->setObjectName(QString::fromUtf8("checkBoxSmooth"));
        checkBoxSmooth->setGeometry(QRect(170, 30, 81, 23));
        checkBoxAxis = new QCheckBox(groupBoxScene);
        checkBoxAxis->setObjectName(QString::fromUtf8("checkBoxAxis"));
        checkBoxAxis->setGeometry(QRect(170, 60, 81, 23));
        groupBoxMPlanning = new QGroupBox(ViewerSettings);
        groupBoxMPlanning->setObjectName(QString::fromUtf8("groupBoxMPlanning"));
        groupBoxMPlanning->setGeometry(QRect(290, 0, 141, 91));
        checkBoxDrawGraph = new QCheckBox(groupBoxMPlanning);
        checkBoxDrawGraph->setObjectName(QString::fromUtf8("checkBoxDrawGraph"));
        checkBoxDrawGraph->setGeometry(QRect(10, 30, 111, 21));
        checkBoxDrawTraj = new QCheckBox(groupBoxMPlanning);
        checkBoxDrawTraj->setObjectName(QString::fromUtf8("checkBoxDrawTraj"));
        checkBoxDrawTraj->setGeometry(QRect(10, 60, 87, 21));
        groupBox3DModel = new QGroupBox(ViewerSettings);
        groupBox3DModel->setObjectName(QString::fromUtf8("groupBox3DModel"));
        groupBox3DModel->setGeometry(QRect(450, 0, 161, 91));
        checkBoxBB = new QCheckBox(groupBox3DModel);
        checkBoxBB->setObjectName(QString::fromUtf8("checkBoxBB"));
        checkBoxBB->setGeometry(QRect(10, 30, 141, 21));
        checkBoxGhosts = new QCheckBox(groupBox3DModel);
        checkBoxGhosts->setObjectName(QString::fromUtf8("checkBoxGhosts"));
        checkBoxGhosts->setGeometry(QRect(10, 60, 101, 21));
        checkBoxFilaire = new QCheckBox(groupBox3DModel);
        checkBoxFilaire->setObjectName(QString::fromUtf8("checkBoxFilaire"));
        checkBoxFilaire->setGeometry(QRect(90, 60, 96, 23));
        toolBox->addItem(ViewerSettings, QString::fromUtf8("Viewer Settings 1"));
        Viewersettings2 = new QWidget();
        Viewersettings2->setObjectName(QString::fromUtf8("Viewersettings2"));
        Viewersettings2->setGeometry(QRect(0, 0, 783, 110));
        doubleSpinBoxLightZ = new QDoubleSpinBox(Viewersettings2);
        doubleSpinBoxLightZ->setObjectName(QString::fromUtf8("doubleSpinBoxLightZ"));
        doubleSpinBoxLightZ->setGeometry(QRect(20, 60, 61, 21));
        doubleSpinBoxLightX = new QDoubleSpinBox(Viewersettings2);
        doubleSpinBoxLightX->setObjectName(QString::fromUtf8("doubleSpinBoxLightX"));
        doubleSpinBoxLightX->setGeometry(QRect(20, 0, 61, 21));
        doubleSpinBoxLightY = new QDoubleSpinBox(Viewersettings2);
        doubleSpinBoxLightY->setObjectName(QString::fromUtf8("doubleSpinBoxLightY"));
        doubleSpinBoxLightY->setGeometry(QRect(20, 30, 61, 21));
        pushButtonRestoreView = new QPushButton(Viewersettings2);
        pushButtonRestoreView->setObjectName(QString::fromUtf8("pushButtonRestoreView"));
        pushButtonRestoreView->setGeometry(QRect(470, 0, 113, 32));
        horizontalSliderLightX = new QSlider(Viewersettings2);
        horizontalSliderLightX->setObjectName(QString::fromUtf8("horizontalSliderLightX"));
        horizontalSliderLightX->setGeometry(QRect(130, 0, 311, 22));
        horizontalSliderLightX->setMaximum(1000);
        horizontalSliderLightX->setOrientation(Qt::Horizontal);
        horizontalSliderLightY = new QSlider(Viewersettings2);
        horizontalSliderLightY->setObjectName(QString::fromUtf8("horizontalSliderLightY"));
        horizontalSliderLightY->setGeometry(QRect(130, 30, 311, 22));
        horizontalSliderLightY->setMaximum(1000);
        horizontalSliderLightY->setOrientation(Qt::Horizontal);
        horizontalSliderLightZ = new QSlider(Viewersettings2);
        horizontalSliderLightZ->setObjectName(QString::fromUtf8("horizontalSliderLightZ"));
        horizontalSliderLightZ->setGeometry(QRect(130, 60, 311, 22));
        horizontalSliderLightZ->setMaximum(1000);
        horizontalSliderLightZ->setOrientation(Qt::Horizontal);
        label_2 = new QLabel(Viewersettings2);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(100, 0, 21, 17));
        label_3 = new QLabel(Viewersettings2);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(100, 30, 21, 17));
        label_4 = new QLabel(Viewersettings2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(100, 60, 21, 17));
        pushButtonRestoreLight = new QPushButton(Viewersettings2);
        pushButtonRestoreLight->setObjectName(QString::fromUtf8("pushButtonRestoreLight"));
        pushButtonRestoreLight->setGeometry(QRect(470, 30, 113, 32));
        checkBoxDrawLightSource = new QCheckBox(Viewersettings2);
        checkBoxDrawLightSource->setObjectName(QString::fromUtf8("checkBoxDrawLightSource"));
        checkBoxDrawLightSource->setGeometry(QRect(480, 60, 141, 21));
        toolBox->addItem(Viewersettings2, QString::fromUtf8("Viewer Settings 2"));
        page = new QWidget();
        page->setObjectName(QString::fromUtf8("page"));
        page->setGeometry(QRect(0, 0, 96, 26));
        pushButtonShowTraj = new QPushButton(page);
        pushButtonShowTraj->setObjectName(QString::fromUtf8("pushButtonShowTraj"));
        pushButtonShowTraj->setGeometry(QRect(10, 0, 141, 32));
        lineEdit = new QLineEdit(page);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        lineEdit->setGeometry(QRect(100, 50, 51, 22));
        horizontalScrollBar = new QScrollBar(page);
        horizontalScrollBar->setObjectName(QString::fromUtf8("horizontalScrollBar"));
        horizontalScrollBar->setGeometry(QRect(170, 50, 231, 20));
        horizontalScrollBar->setOrientation(Qt::Horizontal);
        label = new QLabel(page);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 50, 61, 17));
        toolBox->addItem(page, QString::fromUtf8("Show Trajectory"));

        verticalLayout->addWidget(toolBox);


        horizontalLayout_2->addLayout(verticalLayout);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1319, 22));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        menuCollisionCheker = new QMenu(menuBar);
        menuCollisionCheker->setObjectName(QString::fromUtf8("menuCollisionCheker"));
        menuTrajectory = new QMenu(menuBar);
        menuTrajectory->setObjectName(QString::fromUtf8("menuTrajectory"));
        menuEnvironement_2 = new QMenu(menuBar);
        menuEnvironement_2->setObjectName(QString::fromUtf8("menuEnvironement_2"));
        MainWindow->setMenuBar(menuBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuEnvironement_2->menuAction());
        menuBar->addAction(menuTrajectory->menuAction());
        menuBar->addAction(menuCollisionCheker->menuAction());
        menuBar->addAction(menuHelp->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionCloseEnvironement);
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);
        menuHelp->addAction(actionAbout);
        menuCollisionCheker->addAction(actionKCDPropietes);
        menuEnvironement_2->addAction(actionOpenScenario);
        menuEnvironement_2->addAction(actionSaveScenarion);
        menuEnvironement_2->addSeparator();
        menuEnvironement_2->addAction(actionRobot);
        menuEnvironement_2->addAction(action3DViewer);

        retranslateUi(MainWindow);
        QObject::connect(actionQuit, SIGNAL(triggered()), MainWindow, SLOT(close()));

        toolBox->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionQuit->setText(QApplication::translate("MainWindow", "Quit", 0, QApplication::UnicodeUTF8));
        actionRobot->setText(QApplication::translate("MainWindow", "Robot", 0, QApplication::UnicodeUTF8));
        action3DViewer->setText(QApplication::translate("MainWindow", "3D-Viewer", 0, QApplication::UnicodeUTF8));
        actionKCDPropietes->setText(QApplication::translate("MainWindow", "KCD Propietes", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindow", "About", 0, QApplication::UnicodeUTF8));
        actionCloseEnvironement->setText(QApplication::translate("MainWindow", "Close", 0, QApplication::UnicodeUTF8));
        actionOpenScenario->setText(QApplication::translate("MainWindow", "Load Scenario", 0, QApplication::UnicodeUTF8));
        actionOpen->setText(QApplication::translate("MainWindow", "Open", 0, QApplication::UnicodeUTF8));
        actionSaveScenarion->setText(QApplication::translate("MainWindow", "Save Scenario", 0, QApplication::UnicodeUTF8));
        actionNameOfEnv->setText(QApplication::translate("MainWindow", "NameOfEnv", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "3D View", 0, QApplication::UnicodeUTF8));
        pushButtonRun->setText(QApplication::translate("MainWindow", "Run", 0, QApplication::UnicodeUTF8));
        pushButtonStop->setText(QApplication::translate("MainWindow", "Stop", 0, QApplication::UnicodeUTF8));
        pushButtonReset->setText(QApplication::translate("MainWindow", "Reset", 0, QApplication::UnicodeUTF8));
        checkBoxWithShortCut->setText(QApplication::translate("MainWindow", "With Short Cut", 0, QApplication::UnicodeUTF8));
        radioButtonDiff->setText(QApplication::translate("MainWindow", "Diffusion", 0, QApplication::UnicodeUTF8));
        radioButtonPRM->setText(QApplication::translate("MainWindow", "PRM", 0, QApplication::UnicodeUTF8));
        pushButtonResetGraph->setText(QApplication::translate("MainWindow", "Reset Graph", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(RunMotion), QApplication::translate("MainWindow", "Run Motion Planning", 0, QApplication::UnicodeUTF8));
        groupBoxScene->setTitle(QApplication::translate("MainWindow", "Scene", 0, QApplication::UnicodeUTF8));
        checkBoxTiles->setText(QApplication::translate("MainWindow", "Tiles", 0, QApplication::UnicodeUTF8));
        checkBoxWalls->setText(QApplication::translate("MainWindow", "Walls", 0, QApplication::UnicodeUTF8));
        checkBoxFloor->setText(QApplication::translate("MainWindow", "Floor", 0, QApplication::UnicodeUTF8));
        checkBoxShadows->setText(QApplication::translate("MainWindow", "Shadow", 0, QApplication::UnicodeUTF8));
        checkBoxSmooth->setText(QApplication::translate("MainWindow", "Smooth", 0, QApplication::UnicodeUTF8));
        checkBoxAxis->setText(QApplication::translate("MainWindow", "Axis", 0, QApplication::UnicodeUTF8));
        groupBoxMPlanning->setTitle(QApplication::translate("MainWindow", "Motion Planning", 0, QApplication::UnicodeUTF8));
        checkBoxDrawGraph->setText(QApplication::translate("MainWindow", "Draw Graph", 0, QApplication::UnicodeUTF8));
        checkBoxDrawTraj->setText(QApplication::translate("MainWindow", "Draw Traj", 0, QApplication::UnicodeUTF8));
        groupBox3DModel->setTitle(QApplication::translate("MainWindow", "3D Model", 0, QApplication::UnicodeUTF8));
        checkBoxBB->setText(QApplication::translate("MainWindow", "Bounding Boxes", 0, QApplication::UnicodeUTF8));
        checkBoxGhosts->setText(QApplication::translate("MainWindow", "Ghosts", 0, QApplication::UnicodeUTF8));
        checkBoxFilaire->setText(QApplication::translate("MainWindow", "Filaire", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(ViewerSettings), QApplication::translate("MainWindow", "Viewer Settings 1", 0, QApplication::UnicodeUTF8));
        pushButtonRestoreView->setText(QApplication::translate("MainWindow", "Restore View", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "X", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Y", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Z", 0, QApplication::UnicodeUTF8));
        pushButtonRestoreLight->setText(QApplication::translate("MainWindow", "Restore Light", 0, QApplication::UnicodeUTF8));
        checkBoxDrawLightSource->setText(QApplication::translate("MainWindow", "Draw Light Source", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(Viewersettings2), QApplication::translate("MainWindow", "Viewer Settings 2", 0, QApplication::UnicodeUTF8));
        pushButtonShowTraj->setText(QApplication::translate("MainWindow", "Show Trajectory", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Speed", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(page), QApplication::translate("MainWindow", "Show Trajectory", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuHelp->setTitle(QApplication::translate("MainWindow", "Help", 0, QApplication::UnicodeUTF8));
        menuCollisionCheker->setTitle(QApplication::translate("MainWindow", "Collision Cheker", 0, QApplication::UnicodeUTF8));
        menuTrajectory->setTitle(QApplication::translate("MainWindow", "Trajectory", 0, QApplication::UnicodeUTF8));
        menuEnvironement_2->setTitle(QApplication::translate("MainWindow", "Environement", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // MAINWINDOW_H
