/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Mon Nov 16 13:09:50 2009
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
#include <QtGui/QScrollBar>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
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
    QTextEdit *consoleOutput;
    QWidget *ViewerSettings;
    QGroupBox *groupBoxScene;
    QCheckBox *checkBoxTiles;
    QCheckBox *checkBoxWalls;
    QCheckBox *checkBoxFloor;
    QCheckBox *checkBoxShadows;
    QGroupBox *groupBoxMPlanning;
    QCheckBox *checkBoxDrawGraph;
    QCheckBox *checkBoxDrawTraj;
    QGroupBox *groupBox3DModel;
    QCheckBox *checkBoxBB;
    QCheckBox *checkBoxGhosts;
    QPushButton *pushButtonRestoreView;
    QWidget *page;
    QPushButton *pushButtonShowTraj;
    QLineEdit *lineEdit;
    QScrollBar *horizontalScrollBar;
    QLabel *label;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuEnvironement;
    QMenu *menuAbout;
    QMenu *menuHelp;
    QMenu *menuCollisionCheker;
    QMenu *menuTrajectory;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->setWindowModality(Qt::NonModal);
        MainWindow->setEnabled(true);
        MainWindow->resize(1319, 704);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(10);
        sizePolicy.setVerticalStretch(10);
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
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy1);
        horizontalLayout_2 = new QHBoxLayout(centralWidget);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setSizeConstraint(QLayout::SetFixedSize);
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
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(OpenGL->sizePolicy().hasHeightForWidth());
        OpenGL->setSizePolicy(sizePolicy2);
        OpenGL->setMinimumSize(QSize(400, 300));

        horizontalLayout_3->addWidget(OpenGL);


        verticalLayout->addWidget(groupBox);

        progressBar = new QProgressBar(centralWidget);
        progressBar->setObjectName(QString::fromUtf8("progressBar"));
        progressBar->setValue(24);

        verticalLayout->addWidget(progressBar);

        toolBox = new QToolBox(centralWidget);
        toolBox->setObjectName(QString::fromUtf8("toolBox"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(toolBox->sizePolicy().hasHeightForWidth());
        toolBox->setSizePolicy(sizePolicy3);
        toolBox->setMinimumSize(QSize(0, 250));
        RunMotion = new QWidget();
        RunMotion->setObjectName(QString::fromUtf8("RunMotion"));
        RunMotion->setGeometry(QRect(0, 0, 783, 145));
        RunMotionPlan = new QWidget(RunMotion);
        RunMotionPlan->setObjectName(QString::fromUtf8("RunMotionPlan"));
        RunMotionPlan->setGeometry(QRect(10, 0, 621, 141));
        sizePolicy1.setHeightForWidth(RunMotionPlan->sizePolicy().hasHeightForWidth());
        RunMotionPlan->setSizePolicy(sizePolicy1);
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

        consoleOutput = new QTextEdit(RunMotionPlan);
        consoleOutput->setObjectName(QString::fromUtf8("consoleOutput"));

        horizontalLayout_4->addWidget(consoleOutput);

        toolBox->addItem(RunMotion, QString::fromUtf8("Run Motion Planning"));
        ViewerSettings = new QWidget();
        ViewerSettings->setObjectName(QString::fromUtf8("ViewerSettings"));
        ViewerSettings->setGeometry(QRect(0, 0, 783, 145));
        groupBoxScene = new QGroupBox(ViewerSettings);
        groupBoxScene->setObjectName(QString::fromUtf8("groupBoxScene"));
        groupBoxScene->setGeometry(QRect(10, 0, 211, 91));
        checkBoxTiles = new QCheckBox(groupBoxScene);
        checkBoxTiles->setObjectName(QString::fromUtf8("checkBoxTiles"));
        checkBoxTiles->setGeometry(QRect(10, 30, 101, 21));
        checkBoxWalls = new QCheckBox(groupBoxScene);
        checkBoxWalls->setObjectName(QString::fromUtf8("checkBoxWalls"));
        checkBoxWalls->setGeometry(QRect(110, 30, 71, 21));
        checkBoxFloor = new QCheckBox(groupBoxScene);
        checkBoxFloor->setObjectName(QString::fromUtf8("checkBoxFloor"));
        checkBoxFloor->setGeometry(QRect(10, 60, 101, 21));
        checkBoxShadows = new QCheckBox(groupBoxScene);
        checkBoxShadows->setObjectName(QString::fromUtf8("checkBoxShadows"));
        checkBoxShadows->setGeometry(QRect(110, 60, 81, 21));
        groupBoxMPlanning = new QGroupBox(ViewerSettings);
        groupBoxMPlanning->setObjectName(QString::fromUtf8("groupBoxMPlanning"));
        groupBoxMPlanning->setGeometry(QRect(260, 0, 131, 91));
        checkBoxDrawGraph = new QCheckBox(groupBoxMPlanning);
        checkBoxDrawGraph->setObjectName(QString::fromUtf8("checkBoxDrawGraph"));
        checkBoxDrawGraph->setGeometry(QRect(10, 30, 111, 21));
        checkBoxDrawTraj = new QCheckBox(groupBoxMPlanning);
        checkBoxDrawTraj->setObjectName(QString::fromUtf8("checkBoxDrawTraj"));
        checkBoxDrawTraj->setGeometry(QRect(10, 60, 87, 21));
        groupBox3DModel = new QGroupBox(ViewerSettings);
        groupBox3DModel->setObjectName(QString::fromUtf8("groupBox3DModel"));
        groupBox3DModel->setGeometry(QRect(420, 0, 151, 91));
        checkBoxBB = new QCheckBox(groupBox3DModel);
        checkBoxBB->setObjectName(QString::fromUtf8("checkBoxBB"));
        checkBoxBB->setGeometry(QRect(10, 30, 141, 21));
        checkBoxGhosts = new QCheckBox(groupBox3DModel);
        checkBoxGhosts->setObjectName(QString::fromUtf8("checkBoxGhosts"));
        checkBoxGhosts->setGeometry(QRect(10, 60, 101, 21));
        pushButtonRestoreView = new QPushButton(ViewerSettings);
        pushButtonRestoreView->setObjectName(QString::fromUtf8("pushButtonRestoreView"));
        pushButtonRestoreView->setGeometry(QRect(610, 30, 113, 32));
        toolBox->addItem(ViewerSettings, QString::fromUtf8("Viewer Settings"));
        page = new QWidget();
        page->setObjectName(QString::fromUtf8("page"));
        page->setGeometry(QRect(0, 0, 783, 145));
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
        menuEnvironement = new QMenu(menuFile);
        menuEnvironement->setObjectName(QString::fromUtf8("menuEnvironement"));
        menuAbout = new QMenu(menuBar);
        menuAbout->setObjectName(QString::fromUtf8("menuAbout"));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        menuCollisionCheker = new QMenu(menuBar);
        menuCollisionCheker->setObjectName(QString::fromUtf8("menuCollisionCheker"));
        menuTrajectory = new QMenu(menuBar);
        menuTrajectory->setObjectName(QString::fromUtf8("menuTrajectory"));
        MainWindow->setMenuBar(menuBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuAbout->menuAction());
        menuBar->addAction(menuTrajectory->menuAction());
        menuBar->addAction(menuCollisionCheker->menuAction());
        menuBar->addAction(menuHelp->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionCloseEnvironement);
        menuFile->addAction(menuEnvironement->menuAction());
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);
        menuEnvironement->addAction(actionNameOfEnv);
        menuAbout->addAction(actionOpenScenario);
        menuAbout->addAction(actionSaveScenarion);
        menuAbout->addSeparator();
        menuAbout->addAction(actionRobot);
        menuAbout->addAction(action3DViewer);
        menuHelp->addAction(actionAbout);
        menuCollisionCheker->addAction(actionKCDPropietes);

        retranslateUi(MainWindow);
        QObject::connect(actionQuit, SIGNAL(triggered()), MainWindow, SLOT(close()));

        toolBox->setCurrentIndex(0);


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
        toolBox->setItemText(toolBox->indexOf(RunMotion), QApplication::translate("MainWindow", "Run Motion Planning", 0, QApplication::UnicodeUTF8));
        groupBoxScene->setTitle(QApplication::translate("MainWindow", "Scene", 0, QApplication::UnicodeUTF8));
        checkBoxTiles->setText(QApplication::translate("MainWindow", "Tiles", 0, QApplication::UnicodeUTF8));
        checkBoxWalls->setText(QApplication::translate("MainWindow", "Walls", 0, QApplication::UnicodeUTF8));
        checkBoxFloor->setText(QApplication::translate("MainWindow", "Floor", 0, QApplication::UnicodeUTF8));
        checkBoxShadows->setText(QApplication::translate("MainWindow", "Shadows", 0, QApplication::UnicodeUTF8));
        groupBoxMPlanning->setTitle(QApplication::translate("MainWindow", "Motion Planning", 0, QApplication::UnicodeUTF8));
        checkBoxDrawGraph->setText(QApplication::translate("MainWindow", "Draw Graph", 0, QApplication::UnicodeUTF8));
        checkBoxDrawTraj->setText(QApplication::translate("MainWindow", "Draw Traj", 0, QApplication::UnicodeUTF8));
        groupBox3DModel->setTitle(QApplication::translate("MainWindow", "3D Model", 0, QApplication::UnicodeUTF8));
        checkBoxBB->setText(QApplication::translate("MainWindow", "Bounding Boxes", 0, QApplication::UnicodeUTF8));
        checkBoxGhosts->setText(QApplication::translate("MainWindow", "Ghosts", 0, QApplication::UnicodeUTF8));
        pushButtonRestoreView->setText(QApplication::translate("MainWindow", "Restore View", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(ViewerSettings), QApplication::translate("MainWindow", "Viewer Settings", 0, QApplication::UnicodeUTF8));
        pushButtonShowTraj->setText(QApplication::translate("MainWindow", "Show Trajectory", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Speed", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(page), QApplication::translate("MainWindow", "Show Trajectory", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuEnvironement->setTitle(QApplication::translate("MainWindow", "Environement", 0, QApplication::UnicodeUTF8));
        menuAbout->setTitle(QApplication::translate("MainWindow", "Environement", 0, QApplication::UnicodeUTF8));
        menuHelp->setTitle(QApplication::translate("MainWindow", "Help", 0, QApplication::UnicodeUTF8));
        menuCollisionCheker->setTitle(QApplication::translate("MainWindow", "Collision Cheker", 0, QApplication::UnicodeUTF8));
        menuTrajectory->setTitle(QApplication::translate("MainWindow", "Trajectory", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // MAINWINDOW_H
