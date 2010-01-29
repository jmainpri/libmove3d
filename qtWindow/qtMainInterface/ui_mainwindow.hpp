/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Thu Jan 28 00:08:51 2010
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
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QSplitter>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "../qtFormRobot/moverobot.hpp"
#include "../qtOpenGL/glwidget.hpp"

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
    QVBoxLayout *verticalLayout_29;
    QSplitter *splitter;
    QWidget *sidePanel;
    QVBoxLayout *verticalLayout_6;
    QTabWidget *tabWidget_2;
    QWidget *tabMotionPlanner;
    QVBoxLayout *verticalLayout_19;
    QTabWidget *tabWidget_4;
    QWidget *tabDiffu;
    QVBoxLayout *verticalLayout_39;
    QGridLayout *gridLayout_9;
    QCheckBox *isWithGoal;
    QCheckBox *isBidir;
    QCheckBox *isBalanced;
    QCheckBox *isExpandControl;
    QCheckBox *isManhattan;
    QCheckBox *isDiscardingNodes;
    QCheckBox *isCostSpace;
    QCheckBox *isCostTransition;
    QCheckBox *isEST;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_30;
    QComboBox *expansionMethod;
    QHBoxLayout *horizontalLayout_20;
    QCheckBox *isManualRefiRadius;
    QCheckBox *isAddingCycles;
    QGridLayout *gridLayout_10;
    QLabel *label_5;
    QDoubleSpinBox *doubleSpinBoxExtentionStep;
    QSlider *horizontalSliderExtentionStep;
    QCheckBox *checkBoxIsGoalBias;
    QDoubleSpinBox *doubleSpinBoxBias;
    QSlider *horizontalSliderBias;
    QGridLayout *gridLayout_16;
    QLabel *labelMaxNodes;
    QSpinBox *spinBoxMaxNodes;
    QSpacerItem *horizontalSpacer_21;
    QSpacerItem *verticalSpacer_7;
    QWidget *tabPRM;
    QVBoxLayout *verticalLayout_25;
    QHBoxLayout *horizontalLayout_19;
    QGroupBox *groupBox_17;
    QVBoxLayout *verticalLayout_28;
    QComboBox *comboBox_2;
    QCheckBox *checkBox_9;
    QGroupBox *groupBox_18;
    QVBoxLayout *verticalLayout_27;
    QComboBox *comboBox_3;
    QGroupBox *groupBox_16;
    QGridLayout *gridLayout_8;
    QRadioButton *radioButton;
    QRadioButton *radioButton_3;
    QRadioButton *radioButton_2;
    QRadioButton *radioButton_4;
    QRadioButton *radioButton_5;
    QSpacerItem *verticalSpacer_3;
    QWidget *tabOpti;
    QVBoxLayout *verticalLayout_22;
    QVBoxLayout *optimizeLayout;
    QWidget *tabMultiRun;
    QVBoxLayout *verticalLayout_20;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_17;
    QLineEdit *lineEditContext;
    QPushButton *pushButtonSaveContext;
    QHBoxLayout *horizontalLayout_13;
    QPushButton *pushButtonResetContext;
    QPushButton *pushButtonPrintAllContext;
    QPushButton *pushButtonPrintSelected;
    QWidget *widget_2;
    QVBoxLayout *verticalLayout_21;
    QVBoxLayout *multiRunLayout;
    QGridLayout *gridLayout_7;
    QPushButton *pushButtonSetSelected;
    QPushButton *pushButtonRunAllRRT;
    QPushButton *pushButtonRunAllGreedy;
    QLabel *label_18;
    QSlider *horizontalSliderNbRounds;
    QPushButton *pushButtonShowHisto;
    QDoubleSpinBox *doubleSpinBoxNbRounds;
    QWidget *tabCost;
    QVBoxLayout *verticalLayout_11;
    QTabWidget *tabWidget_5;
    QWidget *tab_12;
    QVBoxLayout *verticalLayout_9;
    QCheckBox *isCostSpaceCopy;
    QGroupBox *groupBox_7;
    QHBoxLayout *horizontalLayout_9;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *pushButtonShowTrajCost;
    QCheckBox *checkBoxRescale;
    QSpacerItem *horizontalSpacer_2;
    QGroupBox *groupBox_10;
    QVBoxLayout *verticalLayout_10;
    QGridLayout *gridLayout_4;
    QLabel *label_6;
    QDoubleSpinBox *doubleSpinBoxInitTemp;
    QSlider *horizontalSliderInitTemp;
    QLabel *label_7;
    QDoubleSpinBox *doubleSpinBoxNFailMax;
    QSlider *horizontalSliderNFailMax;
    QHBoxLayout *horizontalLayout_10;
    QSpacerItem *horizontalSpacer_4;
    QCheckBox *checkBoxCostBefore;
    QPushButton *pushButtonShowTemp;
    QSpacerItem *horizontalSpacer_5;
    QHBoxLayout *horizontalLayout_14;
    QPushButton *pushButtonGridInGraph;
    QPushButton *pushButtonAStar;
    QSpacerItem *horizontalSpacer_15;
    QSpacerItem *verticalSpacer_2;
    QWidget *tab_8;
    QVBoxLayout *verticalLayout_3;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_24;
    QCheckBox *enableHri_2;
    QGroupBox *groupBox_9;
    QVBoxLayout *verticalLayout_33;
    QCheckBox *checkBoxDrawDistance;
    QCheckBox *checkBoxDrawGrid;
    QCheckBox *checkBoxDrawRandPoints;
    QHBoxLayout *horizontalLayout;
    QLabel *label_8;
    QSpinBox *spinBoxJoint;
    QPushButton *pushButtonHRITS;
    QGroupBox *HRITaskSpace;
    QVBoxLayout *verticalLayout_23;
    QHBoxLayout *horizontalLayout_15;
    QCheckBox *enableHriTS;
    QSpacerItem *horizontalSpacer_6;
    QHBoxLayout *horizontalLayout_16;
    QPushButton *pushButtonWorkspacePath;
    QPushButton *pushButtonHoleMotion;
    QSpacerItem *horizontalSpacer_17;
    QGroupBox *groupBox_8;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_5;
    QSpacerItem *horizontalSpacer_16;
    QPushButton *pushButtonMakeGrid;
    QPushButton *pushButtonDeleteGrid;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_9;
    QDoubleSpinBox *doubleSpinBoxCellSize;
    QSlider *horizontalSliderCellSize;
    QGroupBox *HRICSPlanner;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_7;
    QCheckBox *checkBoxHRICS_MOPL;
    QPushButton *pushButtonComputeCost;
    QPushButton *pushButtonResetCost;
    QGridLayout *gridLayout;
    QLabel *label_10;
    QDoubleSpinBox *doubleSpinBoxZoneSize;
    QSlider *horizontalSliderZoneSize;
    QGridLayout *gridLayout_2;
    QPushButton *pushButtonHRICSRRT;
    QPushButton *pushButtonAStaIn3DGrid;
    QCheckBox *checkBoxInverseKinematics;
    QPushButton *pushButtonResetRandPoints;
    QCheckBox *checkBoxHRIGoalBiased;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_14;
    QCheckBox *checkBoxEnableHRIConfigSpace;
    QWidget *widget_5;
    QGridLayout *gridLayout_17;
    QLabel *label_20;
    QDoubleSpinBox *doubleSpinBoxVisibility;
    QSlider *horizontalSliderVisibility;
    QLabel *label_19;
    QDoubleSpinBox *doubleSpinBoxDistance;
    QSlider *horizontalSliderDistance;
    QWidget *widget_7;
    QGridLayout *gridLayout_19;
    QComboBox *whichTestBox;
    QPushButton *pushButtonCreateGrid;
    QWidget *widget_6;
    QGridLayout *gridLayout_18;
    QPushButton *pushButtonNewHRICSpace;
    QPushButton *pushButtonDeleteHRICSpace;
    QCheckBox *checkBoxBBDist;
    QCheckBox *checkBoxBallDist;
    QWidget *tab_11;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_16;
    QDoubleSpinBox *doubleSpinBoxNatural;
    QSlider *horizontalSliderNatural;
    QGroupBox *groupBox_6;
    QVBoxLayout *verticalLayout_8;
    QGridLayout *gridLayout_3;
    QLabel *label_13;
    QDoubleSpinBox *doubleSpinBoxJointLimit;
    QLabel *label_14;
    QDoubleSpinBox *doubleSpinBoxTaskDist;
    QSlider *horizontalSliderTaskDist;
    QSlider *horizontalSliderJointLimit;
    QLabel *label_15;
    QDoubleSpinBox *doubleSpinBoxHeight;
    QSlider *horizontalSliderHeight;
    QSpacerItem *verticalSpacer;
    QWidget *tabRobot;
    QVBoxLayout *verticalLayout_12;
    QTabWidget *tabWidget_6;
    QWidget *tab_7;
    QVBoxLayout *verticalLayout_13;
    MoveRobot *widget;
    QWidget *tab_13;
    QVBoxLayout *verticalLayout_14;
    QGroupBox *groupBox_11;
    QVBoxLayout *verticalLayout_15;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer_7;
    QPushButton *pushButtonTestAll;
    QSpacerItem *horizontalSpacer_8;
    QFrame *line;
    QGridLayout *gridLayout_5;
    QPushButton *pushButtonCollision;
    QPushButton *pushButtonLocalPath;
    QPushButton *pushButtonCost;
    QLabel *labelCollision;
    QLabel *labelLocalPath;
    QLabel *labelTimeCost;
    QSpacerItem *horizontalSpacer_10;
    QSpacerItem *horizontalSpacer_9;
    QGroupBox *groupBox_12;
    QVBoxLayout *verticalLayout_16;
    QHBoxLayout *horizontalLayout_11;
    QSpacerItem *horizontalSpacer_11;
    QPushButton *pushButtonAttMat;
    QSpacerItem *horizontalSpacer_12;
    QGroupBox *groupBoxGrabObject;
    QGridLayout *gridLayout_15;
    QPushButton *pushButtonGrabObject;
    QComboBox *comboBoxGrabObject;
    QPushButton *pushButtonReleaseObject;
    QSpacerItem *verticalSpacer_5;
    QWidget *tabUtils;
    QVBoxLayout *verticalLayout_17;
    QTabWidget *tabWidget_3;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout_18;
    QGridLayout *gridLayout_6;
    QCheckBox *checkBoxDebug;
    QCheckBox *checkBoxRecomputeTrajCost;
    QCheckBox *checkBoxWithShortCut_2;
    QCheckBox *checkBoxUseTRRT;
    QSpacerItem *horizontalSpacer_13;
    QSpacerItem *horizontalSpacer_14;
    QVBoxLayout *greedyLayout;
    QSpacerItem *verticalSpacer_8;
    QWidget *tabViewerSettings;
    QVBoxLayout *verticalLayout_31;
    QHBoxLayout *horizontalLayout_22;
    QGroupBox *groupBox_5;
    QGridLayout *gridLayout_11;
    QCheckBox *checkBoxTiles;
    QCheckBox *checkBoxWalls;
    QCheckBox *checkBoxSmooth;
    QCheckBox *checkBoxFloor;
    QCheckBox *checkBoxShadows;
    QCheckBox *checkBoxAxis;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout_12;
    QCheckBox *checkBoxDrawGraph;
    QCheckBox *checkBoxDrawTraj;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_13;
    QCheckBox *checkBoxBB;
    QCheckBox *checkBoxGhosts;
    QCheckBox *checkBoxFilaire;
    QGroupBox *groupBox_13;
    QGridLayout *gridLayout_21;
    QPushButton *pushButtonRestoreLight;
    QDoubleSpinBox *doubleSpinBoxLightX;
    QLabel *label_2;
    QSlider *horizontalSliderLightX;
    QSlider *horizontalSliderLightY;
    QDoubleSpinBox *doubleSpinBoxLightY;
    QLabel *label_3;
    QDoubleSpinBox *doubleSpinBoxLightZ;
    QLabel *label_4;
    QSlider *horizontalSliderLightZ;
    QCheckBox *checkBoxDrawLightSource;
    QWidget *widget_8;
    QGridLayout *gridLayout_20;
    QSpacerItem *horizontalSpacer_18;
    QPushButton *pushButtonRestoreView;
    QSpacerItem *verticalSpacer_4;
    QWidget *verticalLayout;
    QVBoxLayout *verticalLayout_2;
    QSplitter *splitter_2;
    GLWidget *OpenGL;
    QWidget *widget_3;
    QHBoxLayout *horizontalLayout_3;
    QGroupBox *groupBox_20;
    QVBoxLayout *verticalLayout_32;
    QWidget *widget_4;
    QPushButton *pushButtonReset;
    QPushButton *pushButtonResetGraph;
    QPushButton *pushButtonRun;
    QPushButton *pushButtonStop;
    QHBoxLayout *horizontalLayout_24;
    QRadioButton *radioButtonDiff;
    QRadioButton *radioButtonPRM;
    QCheckBox *checkBoxWithShortCut;
    QGroupBox *groupBox_19;
    QVBoxLayout *verticalLayout_34;
    QHBoxLayout *horizontalLayout_4;
    QSpacerItem *horizontalSpacer_19;
    QPushButton *pushButtonShowTraj;
    QSpacerItem *horizontalSpacer_20;
    QHBoxLayout *horizontalLayout_25;
    QLabel *label;
    QDoubleSpinBox *doubleSpinBox;
    QSlider *horizontalSlider;
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
        MainWindow->resize(1482, 928);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setSizeIncrement(QSize(0, 0));
        MainWindow->setTabShape(QTabWidget::Rounded);
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
        verticalLayout_29 = new QVBoxLayout(centralWidget);
        verticalLayout_29->setSpacing(6);
        verticalLayout_29->setContentsMargins(11, 11, 11, 11);
        verticalLayout_29->setObjectName(QString::fromUtf8("verticalLayout_29"));
        verticalLayout_29->setContentsMargins(-1, 0, -1, 2);
        splitter = new QSplitter(centralWidget);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        sidePanel = new QWidget(splitter);
        sidePanel->setObjectName(QString::fromUtf8("sidePanel"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(2);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(sidePanel->sizePolicy().hasHeightForWidth());
        sidePanel->setSizePolicy(sizePolicy1);
        verticalLayout_6 = new QVBoxLayout(sidePanel);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(-1, 0, -1, 2);
        tabWidget_2 = new QTabWidget(sidePanel);
        tabWidget_2->setObjectName(QString::fromUtf8("tabWidget_2"));
        tabWidget_2->setDocumentMode(true);
        tabMotionPlanner = new QWidget();
        tabMotionPlanner->setObjectName(QString::fromUtf8("tabMotionPlanner"));
        verticalLayout_19 = new QVBoxLayout(tabMotionPlanner);
        verticalLayout_19->setSpacing(6);
        verticalLayout_19->setContentsMargins(11, 11, 11, 11);
        verticalLayout_19->setObjectName(QString::fromUtf8("verticalLayout_19"));
        verticalLayout_19->setContentsMargins(-1, 0, -1, -1);
        tabWidget_4 = new QTabWidget(tabMotionPlanner);
        tabWidget_4->setObjectName(QString::fromUtf8("tabWidget_4"));
        tabWidget_4->setMinimumSize(QSize(500, 0));
        tabWidget_4->setDocumentMode(true);
        tabDiffu = new QWidget();
        tabDiffu->setObjectName(QString::fromUtf8("tabDiffu"));
        verticalLayout_39 = new QVBoxLayout(tabDiffu);
        verticalLayout_39->setSpacing(6);
        verticalLayout_39->setContentsMargins(11, 11, 11, 11);
        verticalLayout_39->setObjectName(QString::fromUtf8("verticalLayout_39"));
        gridLayout_9 = new QGridLayout();
        gridLayout_9->setSpacing(6);
        gridLayout_9->setObjectName(QString::fromUtf8("gridLayout_9"));
        isWithGoal = new QCheckBox(tabDiffu);
        isWithGoal->setObjectName(QString::fromUtf8("isWithGoal"));

        gridLayout_9->addWidget(isWithGoal, 0, 0, 1, 1);

        isBidir = new QCheckBox(tabDiffu);
        isBidir->setObjectName(QString::fromUtf8("isBidir"));

        gridLayout_9->addWidget(isBidir, 0, 1, 1, 1);

        isBalanced = new QCheckBox(tabDiffu);
        isBalanced->setObjectName(QString::fromUtf8("isBalanced"));

        gridLayout_9->addWidget(isBalanced, 0, 2, 1, 1);

        isExpandControl = new QCheckBox(tabDiffu);
        isExpandControl->setObjectName(QString::fromUtf8("isExpandControl"));

        gridLayout_9->addWidget(isExpandControl, 2, 0, 1, 1);

        isManhattan = new QCheckBox(tabDiffu);
        isManhattan->setObjectName(QString::fromUtf8("isManhattan"));

        gridLayout_9->addWidget(isManhattan, 2, 1, 1, 1);

        isDiscardingNodes = new QCheckBox(tabDiffu);
        isDiscardingNodes->setObjectName(QString::fromUtf8("isDiscardingNodes"));

        gridLayout_9->addWidget(isDiscardingNodes, 2, 2, 1, 1);

        isCostSpace = new QCheckBox(tabDiffu);
        isCostSpace->setObjectName(QString::fromUtf8("isCostSpace"));

        gridLayout_9->addWidget(isCostSpace, 3, 0, 1, 1);

        isCostTransition = new QCheckBox(tabDiffu);
        isCostTransition->setObjectName(QString::fromUtf8("isCostTransition"));

        gridLayout_9->addWidget(isCostTransition, 3, 1, 1, 1);

        isEST = new QCheckBox(tabDiffu);
        isEST->setObjectName(QString::fromUtf8("isEST"));

        gridLayout_9->addWidget(isEST, 3, 2, 1, 1);


        verticalLayout_39->addLayout(gridLayout_9);

        groupBox_2 = new QGroupBox(tabDiffu);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        verticalLayout_30 = new QVBoxLayout(groupBox_2);
        verticalLayout_30->setSpacing(6);
        verticalLayout_30->setContentsMargins(11, 11, 11, 11);
        verticalLayout_30->setObjectName(QString::fromUtf8("verticalLayout_30"));
        expansionMethod = new QComboBox(groupBox_2);
        expansionMethod->setObjectName(QString::fromUtf8("expansionMethod"));

        verticalLayout_30->addWidget(expansionMethod);

        horizontalLayout_20 = new QHBoxLayout();
        horizontalLayout_20->setSpacing(6);
        horizontalLayout_20->setObjectName(QString::fromUtf8("horizontalLayout_20"));
        isManualRefiRadius = new QCheckBox(groupBox_2);
        isManualRefiRadius->setObjectName(QString::fromUtf8("isManualRefiRadius"));

        horizontalLayout_20->addWidget(isManualRefiRadius);

        isAddingCycles = new QCheckBox(groupBox_2);
        isAddingCycles->setObjectName(QString::fromUtf8("isAddingCycles"));

        horizontalLayout_20->addWidget(isAddingCycles);


        verticalLayout_30->addLayout(horizontalLayout_20);

        gridLayout_10 = new QGridLayout();
        gridLayout_10->setSpacing(6);
        gridLayout_10->setObjectName(QString::fromUtf8("gridLayout_10"));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_10->addWidget(label_5, 0, 0, 1, 1);

        doubleSpinBoxExtentionStep = new QDoubleSpinBox(groupBox_2);
        doubleSpinBoxExtentionStep->setObjectName(QString::fromUtf8("doubleSpinBoxExtentionStep"));
        doubleSpinBoxExtentionStep->setDecimals(4);
        doubleSpinBoxExtentionStep->setMinimum(0);
        doubleSpinBoxExtentionStep->setMaximum(50);

        gridLayout_10->addWidget(doubleSpinBoxExtentionStep, 0, 1, 1, 1);

        horizontalSliderExtentionStep = new QSlider(groupBox_2);
        horizontalSliderExtentionStep->setObjectName(QString::fromUtf8("horizontalSliderExtentionStep"));
        horizontalSliderExtentionStep->setMaximum(10000000);
        horizontalSliderExtentionStep->setOrientation(Qt::Horizontal);

        gridLayout_10->addWidget(horizontalSliderExtentionStep, 0, 2, 1, 1);

        checkBoxIsGoalBias = new QCheckBox(groupBox_2);
        checkBoxIsGoalBias->setObjectName(QString::fromUtf8("checkBoxIsGoalBias"));

        gridLayout_10->addWidget(checkBoxIsGoalBias, 1, 0, 1, 1);

        doubleSpinBoxBias = new QDoubleSpinBox(groupBox_2);
        doubleSpinBoxBias->setObjectName(QString::fromUtf8("doubleSpinBoxBias"));
        doubleSpinBoxBias->setMaximum(1);
        doubleSpinBoxBias->setSingleStep(0.01);

        gridLayout_10->addWidget(doubleSpinBoxBias, 1, 1, 1, 1);

        horizontalSliderBias = new QSlider(groupBox_2);
        horizontalSliderBias->setObjectName(QString::fromUtf8("horizontalSliderBias"));
        horizontalSliderBias->setOrientation(Qt::Horizontal);

        gridLayout_10->addWidget(horizontalSliderBias, 1, 2, 1, 1);


        verticalLayout_30->addLayout(gridLayout_10);

        gridLayout_16 = new QGridLayout();
        gridLayout_16->setSpacing(6);
        gridLayout_16->setObjectName(QString::fromUtf8("gridLayout_16"));
        labelMaxNodes = new QLabel(groupBox_2);
        labelMaxNodes->setObjectName(QString::fromUtf8("labelMaxNodes"));

        gridLayout_16->addWidget(labelMaxNodes, 0, 0, 1, 1);

        spinBoxMaxNodes = new QSpinBox(groupBox_2);
        spinBoxMaxNodes->setObjectName(QString::fromUtf8("spinBoxMaxNodes"));
        spinBoxMaxNodes->setMaximum(100000000);

        gridLayout_16->addWidget(spinBoxMaxNodes, 0, 1, 1, 1);

        horizontalSpacer_21 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_16->addItem(horizontalSpacer_21, 0, 2, 1, 1);


        verticalLayout_30->addLayout(gridLayout_16);


        verticalLayout_39->addWidget(groupBox_2);

        verticalSpacer_7 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_39->addItem(verticalSpacer_7);

        tabWidget_4->addTab(tabDiffu, QString());
        tabPRM = new QWidget();
        tabPRM->setObjectName(QString::fromUtf8("tabPRM"));
        verticalLayout_25 = new QVBoxLayout(tabPRM);
        verticalLayout_25->setSpacing(6);
        verticalLayout_25->setContentsMargins(11, 11, 11, 11);
        verticalLayout_25->setObjectName(QString::fromUtf8("verticalLayout_25"));
        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setSpacing(6);
        horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
        groupBox_17 = new QGroupBox(tabPRM);
        groupBox_17->setObjectName(QString::fromUtf8("groupBox_17"));
        verticalLayout_28 = new QVBoxLayout(groupBox_17);
        verticalLayout_28->setSpacing(6);
        verticalLayout_28->setContentsMargins(11, 11, 11, 11);
        verticalLayout_28->setObjectName(QString::fromUtf8("verticalLayout_28"));
        comboBox_2 = new QComboBox(groupBox_17);
        comboBox_2->setObjectName(QString::fromUtf8("comboBox_2"));

        verticalLayout_28->addWidget(comboBox_2);

        checkBox_9 = new QCheckBox(groupBox_17);
        checkBox_9->setObjectName(QString::fromUtf8("checkBox_9"));

        verticalLayout_28->addWidget(checkBox_9);


        horizontalLayout_19->addWidget(groupBox_17);

        groupBox_18 = new QGroupBox(tabPRM);
        groupBox_18->setObjectName(QString::fromUtf8("groupBox_18"));
        verticalLayout_27 = new QVBoxLayout(groupBox_18);
        verticalLayout_27->setSpacing(6);
        verticalLayout_27->setContentsMargins(11, 11, 11, 11);
        verticalLayout_27->setObjectName(QString::fromUtf8("verticalLayout_27"));
        comboBox_3 = new QComboBox(groupBox_18);
        comboBox_3->setObjectName(QString::fromUtf8("comboBox_3"));

        verticalLayout_27->addWidget(comboBox_3);


        horizontalLayout_19->addWidget(groupBox_18);


        verticalLayout_25->addLayout(horizontalLayout_19);

        groupBox_16 = new QGroupBox(tabPRM);
        groupBox_16->setObjectName(QString::fromUtf8("groupBox_16"));
        gridLayout_8 = new QGridLayout(groupBox_16);
        gridLayout_8->setSpacing(6);
        gridLayout_8->setContentsMargins(11, 11, 11, 11);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        radioButton = new QRadioButton(groupBox_16);
        radioButton->setObjectName(QString::fromUtf8("radioButton"));

        gridLayout_8->addWidget(radioButton, 0, 0, 1, 1);

        radioButton_3 = new QRadioButton(groupBox_16);
        radioButton_3->setObjectName(QString::fromUtf8("radioButton_3"));

        gridLayout_8->addWidget(radioButton_3, 0, 1, 1, 1);

        radioButton_2 = new QRadioButton(groupBox_16);
        radioButton_2->setObjectName(QString::fromUtf8("radioButton_2"));

        gridLayout_8->addWidget(radioButton_2, 0, 2, 1, 1);

        radioButton_4 = new QRadioButton(groupBox_16);
        radioButton_4->setObjectName(QString::fromUtf8("radioButton_4"));

        gridLayout_8->addWidget(radioButton_4, 1, 0, 1, 1);

        radioButton_5 = new QRadioButton(groupBox_16);
        radioButton_5->setObjectName(QString::fromUtf8("radioButton_5"));

        gridLayout_8->addWidget(radioButton_5, 1, 1, 1, 1);


        verticalLayout_25->addWidget(groupBox_16);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_25->addItem(verticalSpacer_3);

        tabWidget_4->addTab(tabPRM, QString());
        tabOpti = new QWidget();
        tabOpti->setObjectName(QString::fromUtf8("tabOpti"));
        verticalLayout_22 = new QVBoxLayout(tabOpti);
        verticalLayout_22->setSpacing(6);
        verticalLayout_22->setContentsMargins(11, 11, 11, 11);
        verticalLayout_22->setObjectName(QString::fromUtf8("verticalLayout_22"));
        optimizeLayout = new QVBoxLayout();
        optimizeLayout->setSpacing(6);
        optimizeLayout->setObjectName(QString::fromUtf8("optimizeLayout"));

        verticalLayout_22->addLayout(optimizeLayout);

        tabWidget_4->addTab(tabOpti, QString());
        tabMultiRun = new QWidget();
        tabMultiRun->setObjectName(QString::fromUtf8("tabMultiRun"));
        verticalLayout_20 = new QVBoxLayout(tabMultiRun);
        verticalLayout_20->setSpacing(6);
        verticalLayout_20->setContentsMargins(11, 11, 11, 11);
        verticalLayout_20->setObjectName(QString::fromUtf8("verticalLayout_20"));
        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        horizontalLayout_12->setSizeConstraint(QLayout::SetDefaultConstraint);
        label_17 = new QLabel(tabMultiRun);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        horizontalLayout_12->addWidget(label_17);

        lineEditContext = new QLineEdit(tabMultiRun);
        lineEditContext->setObjectName(QString::fromUtf8("lineEditContext"));

        horizontalLayout_12->addWidget(lineEditContext);

        pushButtonSaveContext = new QPushButton(tabMultiRun);
        pushButtonSaveContext->setObjectName(QString::fromUtf8("pushButtonSaveContext"));

        horizontalLayout_12->addWidget(pushButtonSaveContext);


        verticalLayout_20->addLayout(horizontalLayout_12);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setSpacing(6);
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        horizontalLayout_13->setSizeConstraint(QLayout::SetDefaultConstraint);
        pushButtonResetContext = new QPushButton(tabMultiRun);
        pushButtonResetContext->setObjectName(QString::fromUtf8("pushButtonResetContext"));

        horizontalLayout_13->addWidget(pushButtonResetContext);

        pushButtonPrintAllContext = new QPushButton(tabMultiRun);
        pushButtonPrintAllContext->setObjectName(QString::fromUtf8("pushButtonPrintAllContext"));

        horizontalLayout_13->addWidget(pushButtonPrintAllContext);

        pushButtonPrintSelected = new QPushButton(tabMultiRun);
        pushButtonPrintSelected->setObjectName(QString::fromUtf8("pushButtonPrintSelected"));

        horizontalLayout_13->addWidget(pushButtonPrintSelected);


        verticalLayout_20->addLayout(horizontalLayout_13);

        widget_2 = new QWidget(tabMultiRun);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(widget_2->sizePolicy().hasHeightForWidth());
        widget_2->setSizePolicy(sizePolicy2);
        verticalLayout_21 = new QVBoxLayout(widget_2);
        verticalLayout_21->setSpacing(6);
        verticalLayout_21->setContentsMargins(11, 11, 11, 11);
        verticalLayout_21->setObjectName(QString::fromUtf8("verticalLayout_21"));
        multiRunLayout = new QVBoxLayout();
        multiRunLayout->setSpacing(6);
        multiRunLayout->setObjectName(QString::fromUtf8("multiRunLayout"));
        multiRunLayout->setSizeConstraint(QLayout::SetDefaultConstraint);

        verticalLayout_21->addLayout(multiRunLayout);


        verticalLayout_20->addWidget(widget_2);

        gridLayout_7 = new QGridLayout();
        gridLayout_7->setSpacing(6);
        gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
        pushButtonSetSelected = new QPushButton(tabMultiRun);
        pushButtonSetSelected->setObjectName(QString::fromUtf8("pushButtonSetSelected"));

        gridLayout_7->addWidget(pushButtonSetSelected, 0, 2, 1, 1);

        pushButtonRunAllRRT = new QPushButton(tabMultiRun);
        pushButtonRunAllRRT->setObjectName(QString::fromUtf8("pushButtonRunAllRRT"));

        gridLayout_7->addWidget(pushButtonRunAllRRT, 1, 2, 1, 1);

        pushButtonRunAllGreedy = new QPushButton(tabMultiRun);
        pushButtonRunAllGreedy->setObjectName(QString::fromUtf8("pushButtonRunAllGreedy"));

        gridLayout_7->addWidget(pushButtonRunAllGreedy, 2, 2, 1, 1);

        label_18 = new QLabel(tabMultiRun);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        gridLayout_7->addWidget(label_18, 1, 1, 1, 1);

        horizontalSliderNbRounds = new QSlider(tabMultiRun);
        horizontalSliderNbRounds->setObjectName(QString::fromUtf8("horizontalSliderNbRounds"));
        horizontalSliderNbRounds->setOrientation(Qt::Horizontal);

        gridLayout_7->addWidget(horizontalSliderNbRounds, 2, 1, 1, 1);

        pushButtonShowHisto = new QPushButton(tabMultiRun);
        pushButtonShowHisto->setObjectName(QString::fromUtf8("pushButtonShowHisto"));

        gridLayout_7->addWidget(pushButtonShowHisto, 3, 2, 1, 1);

        doubleSpinBoxNbRounds = new QDoubleSpinBox(tabMultiRun);
        doubleSpinBoxNbRounds->setObjectName(QString::fromUtf8("doubleSpinBoxNbRounds"));

        gridLayout_7->addWidget(doubleSpinBoxNbRounds, 2, 0, 1, 1);


        verticalLayout_20->addLayout(gridLayout_7);

        tabWidget_4->addTab(tabMultiRun, QString());

        verticalLayout_19->addWidget(tabWidget_4);

        tabWidget_2->addTab(tabMotionPlanner, QString());
        tabCost = new QWidget();
        tabCost->setObjectName(QString::fromUtf8("tabCost"));
        verticalLayout_11 = new QVBoxLayout(tabCost);
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setContentsMargins(11, 11, 11, 11);
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
        verticalLayout_11->setContentsMargins(-1, 0, -1, -1);
        tabWidget_5 = new QTabWidget(tabCost);
        tabWidget_5->setObjectName(QString::fromUtf8("tabWidget_5"));
        tabWidget_5->setEnabled(true);
        tabWidget_5->setDocumentMode(true);
        tab_12 = new QWidget();
        tab_12->setObjectName(QString::fromUtf8("tab_12"));
        verticalLayout_9 = new QVBoxLayout(tab_12);
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setContentsMargins(11, 11, 11, 11);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        isCostSpaceCopy = new QCheckBox(tab_12);
        isCostSpaceCopy->setObjectName(QString::fromUtf8("isCostSpaceCopy"));

        verticalLayout_9->addWidget(isCostSpaceCopy);

        groupBox_7 = new QGroupBox(tab_12);
        groupBox_7->setObjectName(QString::fromUtf8("groupBox_7"));
        horizontalLayout_9 = new QHBoxLayout(groupBox_7);
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_9->addItem(horizontalSpacer_3);

        pushButtonShowTrajCost = new QPushButton(groupBox_7);
        pushButtonShowTrajCost->setObjectName(QString::fromUtf8("pushButtonShowTrajCost"));

        horizontalLayout_9->addWidget(pushButtonShowTrajCost);

        checkBoxRescale = new QCheckBox(groupBox_7);
        checkBoxRescale->setObjectName(QString::fromUtf8("checkBoxRescale"));

        horizontalLayout_9->addWidget(checkBoxRescale);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_9->addItem(horizontalSpacer_2);


        verticalLayout_9->addWidget(groupBox_7);

        groupBox_10 = new QGroupBox(tab_12);
        groupBox_10->setObjectName(QString::fromUtf8("groupBox_10"));
        verticalLayout_10 = new QVBoxLayout(groupBox_10);
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setContentsMargins(11, 11, 11, 11);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        gridLayout_4 = new QGridLayout();
        gridLayout_4->setSpacing(6);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        label_6 = new QLabel(groupBox_10);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_4->addWidget(label_6, 0, 0, 1, 1);

        doubleSpinBoxInitTemp = new QDoubleSpinBox(groupBox_10);
        doubleSpinBoxInitTemp->setObjectName(QString::fromUtf8("doubleSpinBoxInitTemp"));
        doubleSpinBoxInitTemp->setDecimals(6);
        doubleSpinBoxInitTemp->setMinimum(1e-06);
        doubleSpinBoxInitTemp->setMaximum(10);
        doubleSpinBoxInitTemp->setSingleStep(1e-06);

        gridLayout_4->addWidget(doubleSpinBoxInitTemp, 0, 1, 1, 1);

        horizontalSliderInitTemp = new QSlider(groupBox_10);
        horizontalSliderInitTemp->setObjectName(QString::fromUtf8("horizontalSliderInitTemp"));
        horizontalSliderInitTemp->setMaximum(10000000);
        horizontalSliderInitTemp->setOrientation(Qt::Horizontal);

        gridLayout_4->addWidget(horizontalSliderInitTemp, 0, 2, 1, 1);

        label_7 = new QLabel(groupBox_10);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_4->addWidget(label_7, 1, 0, 1, 1);

        doubleSpinBoxNFailMax = new QDoubleSpinBox(groupBox_10);
        doubleSpinBoxNFailMax->setObjectName(QString::fromUtf8("doubleSpinBoxNFailMax"));
        doubleSpinBoxNFailMax->setDecimals(0);
        doubleSpinBoxNFailMax->setMinimum(10);
        doubleSpinBoxNFailMax->setMaximum(10000);
        doubleSpinBoxNFailMax->setSingleStep(10);

        gridLayout_4->addWidget(doubleSpinBoxNFailMax, 1, 1, 1, 1);

        horizontalSliderNFailMax = new QSlider(groupBox_10);
        horizontalSliderNFailMax->setObjectName(QString::fromUtf8("horizontalSliderNFailMax"));
        horizontalSliderNFailMax->setMaximum(500);
        horizontalSliderNFailMax->setOrientation(Qt::Horizontal);

        gridLayout_4->addWidget(horizontalSliderNFailMax, 1, 2, 1, 1);


        verticalLayout_10->addLayout(gridLayout_4);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_4);

        checkBoxCostBefore = new QCheckBox(groupBox_10);
        checkBoxCostBefore->setObjectName(QString::fromUtf8("checkBoxCostBefore"));

        horizontalLayout_10->addWidget(checkBoxCostBefore);

        pushButtonShowTemp = new QPushButton(groupBox_10);
        pushButtonShowTemp->setObjectName(QString::fromUtf8("pushButtonShowTemp"));

        horizontalLayout_10->addWidget(pushButtonShowTemp);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_5);


        verticalLayout_10->addLayout(horizontalLayout_10);


        verticalLayout_9->addWidget(groupBox_10);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setSpacing(6);
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        pushButtonGridInGraph = new QPushButton(tab_12);
        pushButtonGridInGraph->setObjectName(QString::fromUtf8("pushButtonGridInGraph"));

        horizontalLayout_14->addWidget(pushButtonGridInGraph);

        pushButtonAStar = new QPushButton(tab_12);
        pushButtonAStar->setObjectName(QString::fromUtf8("pushButtonAStar"));

        horizontalLayout_14->addWidget(pushButtonAStar);

        horizontalSpacer_15 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_14->addItem(horizontalSpacer_15);


        verticalLayout_9->addLayout(horizontalLayout_14);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_9->addItem(verticalSpacer_2);

        tabWidget_5->addTab(tab_12, QString());
        tab_8 = new QWidget();
        tab_8->setObjectName(QString::fromUtf8("tab_8"));
        verticalLayout_3 = new QVBoxLayout(tab_8);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        scrollArea = new QScrollArea(tab_8);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 445, 1016));
        verticalLayout_24 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_24->setSpacing(6);
        verticalLayout_24->setContentsMargins(11, 11, 11, 11);
        verticalLayout_24->setObjectName(QString::fromUtf8("verticalLayout_24"));
        enableHri_2 = new QCheckBox(scrollAreaWidgetContents);
        enableHri_2->setObjectName(QString::fromUtf8("enableHri_2"));
        enableHri_2->setMinimumSize(QSize(0, 35));

        verticalLayout_24->addWidget(enableHri_2);

        groupBox_9 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_9->setObjectName(QString::fromUtf8("groupBox_9"));
        verticalLayout_33 = new QVBoxLayout(groupBox_9);
        verticalLayout_33->setSpacing(6);
        verticalLayout_33->setContentsMargins(11, 11, 11, 11);
        verticalLayout_33->setObjectName(QString::fromUtf8("verticalLayout_33"));
        checkBoxDrawDistance = new QCheckBox(groupBox_9);
        checkBoxDrawDistance->setObjectName(QString::fromUtf8("checkBoxDrawDistance"));

        verticalLayout_33->addWidget(checkBoxDrawDistance);

        checkBoxDrawGrid = new QCheckBox(groupBox_9);
        checkBoxDrawGrid->setObjectName(QString::fromUtf8("checkBoxDrawGrid"));

        verticalLayout_33->addWidget(checkBoxDrawGrid);

        checkBoxDrawRandPoints = new QCheckBox(groupBox_9);
        checkBoxDrawRandPoints->setObjectName(QString::fromUtf8("checkBoxDrawRandPoints"));

        verticalLayout_33->addWidget(checkBoxDrawRandPoints);


        verticalLayout_24->addWidget(groupBox_9);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_8 = new QLabel(scrollAreaWidgetContents);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout->addWidget(label_8);

        spinBoxJoint = new QSpinBox(scrollAreaWidgetContents);
        spinBoxJoint->setObjectName(QString::fromUtf8("spinBoxJoint"));

        horizontalLayout->addWidget(spinBoxJoint);

        pushButtonHRITS = new QPushButton(scrollAreaWidgetContents);
        pushButtonHRITS->setObjectName(QString::fromUtf8("pushButtonHRITS"));

        horizontalLayout->addWidget(pushButtonHRITS);


        verticalLayout_24->addLayout(horizontalLayout);

        HRITaskSpace = new QGroupBox(scrollAreaWidgetContents);
        HRITaskSpace->setObjectName(QString::fromUtf8("HRITaskSpace"));
        HRITaskSpace->setCheckable(false);
        verticalLayout_23 = new QVBoxLayout(HRITaskSpace);
        verticalLayout_23->setSpacing(6);
        verticalLayout_23->setContentsMargins(11, 11, 11, 11);
        verticalLayout_23->setObjectName(QString::fromUtf8("verticalLayout_23"));
        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setSpacing(6);
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        enableHriTS = new QCheckBox(HRITaskSpace);
        enableHriTS->setObjectName(QString::fromUtf8("enableHriTS"));

        horizontalLayout_15->addWidget(enableHriTS);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_15->addItem(horizontalSpacer_6);


        verticalLayout_23->addLayout(horizontalLayout_15);

        horizontalLayout_16 = new QHBoxLayout();
        horizontalLayout_16->setSpacing(6);
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        pushButtonWorkspacePath = new QPushButton(HRITaskSpace);
        pushButtonWorkspacePath->setObjectName(QString::fromUtf8("pushButtonWorkspacePath"));

        horizontalLayout_16->addWidget(pushButtonWorkspacePath);

        pushButtonHoleMotion = new QPushButton(HRITaskSpace);
        pushButtonHoleMotion->setObjectName(QString::fromUtf8("pushButtonHoleMotion"));

        horizontalLayout_16->addWidget(pushButtonHoleMotion);

        horizontalSpacer_17 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_16->addItem(horizontalSpacer_17);


        verticalLayout_23->addLayout(horizontalLayout_16);


        verticalLayout_24->addWidget(HRITaskSpace);

        groupBox_8 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_8->setObjectName(QString::fromUtf8("groupBox_8"));
        verticalLayout_4 = new QVBoxLayout(groupBox_8);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalSpacer_16 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_16);

        pushButtonMakeGrid = new QPushButton(groupBox_8);
        pushButtonMakeGrid->setObjectName(QString::fromUtf8("pushButtonMakeGrid"));

        horizontalLayout_5->addWidget(pushButtonMakeGrid);

        pushButtonDeleteGrid = new QPushButton(groupBox_8);
        pushButtonDeleteGrid->setObjectName(QString::fromUtf8("pushButtonDeleteGrid"));

        horizontalLayout_5->addWidget(pushButtonDeleteGrid);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer);


        verticalLayout_4->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_9 = new QLabel(groupBox_8);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        horizontalLayout_6->addWidget(label_9);

        doubleSpinBoxCellSize = new QDoubleSpinBox(groupBox_8);
        doubleSpinBoxCellSize->setObjectName(QString::fromUtf8("doubleSpinBoxCellSize"));
        doubleSpinBoxCellSize->setMaximum(10);
        doubleSpinBoxCellSize->setSingleStep(0.1);

        horizontalLayout_6->addWidget(doubleSpinBoxCellSize);

        horizontalSliderCellSize = new QSlider(groupBox_8);
        horizontalSliderCellSize->setObjectName(QString::fromUtf8("horizontalSliderCellSize"));
        horizontalSliderCellSize->setOrientation(Qt::Horizontal);

        horizontalLayout_6->addWidget(horizontalSliderCellSize);


        verticalLayout_4->addLayout(horizontalLayout_6);


        verticalLayout_24->addWidget(groupBox_8);

        HRICSPlanner = new QGroupBox(scrollAreaWidgetContents);
        HRICSPlanner->setObjectName(QString::fromUtf8("HRICSPlanner"));
        verticalLayout_5 = new QVBoxLayout(HRICSPlanner);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        checkBoxHRICS_MOPL = new QCheckBox(HRICSPlanner);
        checkBoxHRICS_MOPL->setObjectName(QString::fromUtf8("checkBoxHRICS_MOPL"));

        horizontalLayout_7->addWidget(checkBoxHRICS_MOPL);

        pushButtonComputeCost = new QPushButton(HRICSPlanner);
        pushButtonComputeCost->setObjectName(QString::fromUtf8("pushButtonComputeCost"));

        horizontalLayout_7->addWidget(pushButtonComputeCost);

        pushButtonResetCost = new QPushButton(HRICSPlanner);
        pushButtonResetCost->setObjectName(QString::fromUtf8("pushButtonResetCost"));

        horizontalLayout_7->addWidget(pushButtonResetCost);


        verticalLayout_5->addLayout(horizontalLayout_7);

        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_10 = new QLabel(HRICSPlanner);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout->addWidget(label_10, 0, 0, 1, 1);

        doubleSpinBoxZoneSize = new QDoubleSpinBox(HRICSPlanner);
        doubleSpinBoxZoneSize->setObjectName(QString::fromUtf8("doubleSpinBoxZoneSize"));
        doubleSpinBoxZoneSize->setMaximum(2);
        doubleSpinBoxZoneSize->setSingleStep(0.1);

        gridLayout->addWidget(doubleSpinBoxZoneSize, 0, 1, 1, 1);

        horizontalSliderZoneSize = new QSlider(HRICSPlanner);
        horizontalSliderZoneSize->setObjectName(QString::fromUtf8("horizontalSliderZoneSize"));
        horizontalSliderZoneSize->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSliderZoneSize, 0, 2, 1, 1);


        verticalLayout_5->addLayout(gridLayout);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setSpacing(6);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        pushButtonHRICSRRT = new QPushButton(HRICSPlanner);
        pushButtonHRICSRRT->setObjectName(QString::fromUtf8("pushButtonHRICSRRT"));

        gridLayout_2->addWidget(pushButtonHRICSRRT, 6, 0, 1, 1);

        pushButtonAStaIn3DGrid = new QPushButton(HRICSPlanner);
        pushButtonAStaIn3DGrid->setObjectName(QString::fromUtf8("pushButtonAStaIn3DGrid"));

        gridLayout_2->addWidget(pushButtonAStaIn3DGrid, 2, 0, 1, 1);

        checkBoxInverseKinematics = new QCheckBox(HRICSPlanner);
        checkBoxInverseKinematics->setObjectName(QString::fromUtf8("checkBoxInverseKinematics"));

        gridLayout_2->addWidget(checkBoxInverseKinematics, 6, 2, 1, 1);

        pushButtonResetRandPoints = new QPushButton(HRICSPlanner);
        pushButtonResetRandPoints->setObjectName(QString::fromUtf8("pushButtonResetRandPoints"));

        gridLayout_2->addWidget(pushButtonResetRandPoints, 3, 0, 1, 1);

        checkBoxHRIGoalBiased = new QCheckBox(HRICSPlanner);
        checkBoxHRIGoalBiased->setObjectName(QString::fromUtf8("checkBoxHRIGoalBiased"));

        gridLayout_2->addWidget(checkBoxHRIGoalBiased, 7, 2, 1, 1);


        verticalLayout_5->addLayout(gridLayout_2);


        verticalLayout_24->addWidget(HRICSPlanner);

        groupBox = new QGroupBox(scrollAreaWidgetContents);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_14 = new QGridLayout(groupBox);
        gridLayout_14->setSpacing(6);
        gridLayout_14->setContentsMargins(11, 11, 11, 11);
        gridLayout_14->setObjectName(QString::fromUtf8("gridLayout_14"));
        checkBoxEnableHRIConfigSpace = new QCheckBox(groupBox);
        checkBoxEnableHRIConfigSpace->setObjectName(QString::fromUtf8("checkBoxEnableHRIConfigSpace"));

        gridLayout_14->addWidget(checkBoxEnableHRIConfigSpace, 0, 0, 1, 1);

        widget_5 = new QWidget(groupBox);
        widget_5->setObjectName(QString::fromUtf8("widget_5"));
        gridLayout_17 = new QGridLayout(widget_5);
        gridLayout_17->setSpacing(6);
        gridLayout_17->setContentsMargins(11, 11, 11, 11);
        gridLayout_17->setObjectName(QString::fromUtf8("gridLayout_17"));
        label_20 = new QLabel(widget_5);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        gridLayout_17->addWidget(label_20, 1, 0, 1, 1);

        doubleSpinBoxVisibility = new QDoubleSpinBox(widget_5);
        doubleSpinBoxVisibility->setObjectName(QString::fromUtf8("doubleSpinBoxVisibility"));

        gridLayout_17->addWidget(doubleSpinBoxVisibility, 1, 1, 1, 1);

        horizontalSliderVisibility = new QSlider(widget_5);
        horizontalSliderVisibility->setObjectName(QString::fromUtf8("horizontalSliderVisibility"));
        horizontalSliderVisibility->setOrientation(Qt::Horizontal);

        gridLayout_17->addWidget(horizontalSliderVisibility, 1, 2, 1, 1);

        label_19 = new QLabel(widget_5);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        gridLayout_17->addWidget(label_19, 0, 0, 1, 1);

        doubleSpinBoxDistance = new QDoubleSpinBox(widget_5);
        doubleSpinBoxDistance->setObjectName(QString::fromUtf8("doubleSpinBoxDistance"));
        doubleSpinBoxDistance->setMaximum(300);

        gridLayout_17->addWidget(doubleSpinBoxDistance, 0, 1, 1, 1);

        horizontalSliderDistance = new QSlider(widget_5);
        horizontalSliderDistance->setObjectName(QString::fromUtf8("horizontalSliderDistance"));
        horizontalSliderDistance->setMaximum(500);
        horizontalSliderDistance->setOrientation(Qt::Horizontal);

        gridLayout_17->addWidget(horizontalSliderDistance, 0, 2, 1, 1);


        gridLayout_14->addWidget(widget_5, 5, 0, 1, 1);

        widget_7 = new QWidget(groupBox);
        widget_7->setObjectName(QString::fromUtf8("widget_7"));
        gridLayout_19 = new QGridLayout(widget_7);
        gridLayout_19->setSpacing(6);
        gridLayout_19->setContentsMargins(11, 11, 11, 11);
        gridLayout_19->setObjectName(QString::fromUtf8("gridLayout_19"));
        whichTestBox = new QComboBox(widget_7);
        whichTestBox->setObjectName(QString::fromUtf8("whichTestBox"));

        gridLayout_19->addWidget(whichTestBox, 0, 0, 1, 1);

        pushButtonCreateGrid = new QPushButton(widget_7);
        pushButtonCreateGrid->setObjectName(QString::fromUtf8("pushButtonCreateGrid"));

        gridLayout_19->addWidget(pushButtonCreateGrid, 0, 1, 1, 1);


        gridLayout_14->addWidget(widget_7, 4, 0, 1, 1);

        widget_6 = new QWidget(groupBox);
        widget_6->setObjectName(QString::fromUtf8("widget_6"));
        gridLayout_18 = new QGridLayout(widget_6);
        gridLayout_18->setSpacing(6);
        gridLayout_18->setContentsMargins(11, 11, 11, 11);
        gridLayout_18->setObjectName(QString::fromUtf8("gridLayout_18"));
        pushButtonNewHRICSpace = new QPushButton(widget_6);
        pushButtonNewHRICSpace->setObjectName(QString::fromUtf8("pushButtonNewHRICSpace"));

        gridLayout_18->addWidget(pushButtonNewHRICSpace, 0, 0, 1, 1);

        pushButtonDeleteHRICSpace = new QPushButton(widget_6);
        pushButtonDeleteHRICSpace->setObjectName(QString::fromUtf8("pushButtonDeleteHRICSpace"));

        gridLayout_18->addWidget(pushButtonDeleteHRICSpace, 0, 1, 1, 1);


        gridLayout_14->addWidget(widget_6, 3, 0, 1, 1);

        checkBoxBBDist = new QCheckBox(groupBox);
        checkBoxBBDist->setObjectName(QString::fromUtf8("checkBoxBBDist"));

        gridLayout_14->addWidget(checkBoxBBDist, 1, 0, 1, 1);

        checkBoxBallDist = new QCheckBox(groupBox);
        checkBoxBallDist->setObjectName(QString::fromUtf8("checkBoxBallDist"));

        gridLayout_14->addWidget(checkBoxBallDist, 2, 0, 1, 1);


        verticalLayout_24->addWidget(groupBox);

        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout_3->addWidget(scrollArea);

        tabWidget_5->addTab(tab_8, QString());
        tab_11 = new QWidget();
        tab_11->setObjectName(QString::fromUtf8("tab_11"));
        verticalLayout_7 = new QVBoxLayout(tab_11);
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setContentsMargins(11, 11, 11, 11);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_16 = new QLabel(tab_11);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        horizontalLayout_8->addWidget(label_16);

        doubleSpinBoxNatural = new QDoubleSpinBox(tab_11);
        doubleSpinBoxNatural->setObjectName(QString::fromUtf8("doubleSpinBoxNatural"));
        doubleSpinBoxNatural->setDecimals(4);

        horizontalLayout_8->addWidget(doubleSpinBoxNatural);

        horizontalSliderNatural = new QSlider(tab_11);
        horizontalSliderNatural->setObjectName(QString::fromUtf8("horizontalSliderNatural"));
        horizontalSliderNatural->setMaximum(5000);
        horizontalSliderNatural->setOrientation(Qt::Horizontal);

        horizontalLayout_8->addWidget(horizontalSliderNatural);


        verticalLayout_7->addLayout(horizontalLayout_8);

        groupBox_6 = new QGroupBox(tab_11);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        verticalLayout_8 = new QVBoxLayout(groupBox_6);
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setContentsMargins(11, 11, 11, 11);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        gridLayout_3 = new QGridLayout();
        gridLayout_3->setSpacing(6);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        label_13 = new QLabel(groupBox_6);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout_3->addWidget(label_13, 0, 0, 1, 1);

        doubleSpinBoxJointLimit = new QDoubleSpinBox(groupBox_6);
        doubleSpinBoxJointLimit->setObjectName(QString::fromUtf8("doubleSpinBoxJointLimit"));
        doubleSpinBoxJointLimit->setDecimals(4);

        gridLayout_3->addWidget(doubleSpinBoxJointLimit, 0, 1, 1, 1);

        label_14 = new QLabel(groupBox_6);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        gridLayout_3->addWidget(label_14, 1, 0, 1, 1);

        doubleSpinBoxTaskDist = new QDoubleSpinBox(groupBox_6);
        doubleSpinBoxTaskDist->setObjectName(QString::fromUtf8("doubleSpinBoxTaskDist"));
        doubleSpinBoxTaskDist->setDecimals(4);

        gridLayout_3->addWidget(doubleSpinBoxTaskDist, 1, 1, 1, 1);

        horizontalSliderTaskDist = new QSlider(groupBox_6);
        horizontalSliderTaskDist->setObjectName(QString::fromUtf8("horizontalSliderTaskDist"));
        horizontalSliderTaskDist->setMaximum(5000);
        horizontalSliderTaskDist->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(horizontalSliderTaskDist, 1, 2, 1, 1);

        horizontalSliderJointLimit = new QSlider(groupBox_6);
        horizontalSliderJointLimit->setObjectName(QString::fromUtf8("horizontalSliderJointLimit"));
        horizontalSliderJointLimit->setMaximum(5000);
        horizontalSliderJointLimit->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(horizontalSliderJointLimit, 0, 2, 1, 1);

        label_15 = new QLabel(groupBox_6);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        gridLayout_3->addWidget(label_15, 2, 0, 1, 1);

        doubleSpinBoxHeight = new QDoubleSpinBox(groupBox_6);
        doubleSpinBoxHeight->setObjectName(QString::fromUtf8("doubleSpinBoxHeight"));
        doubleSpinBoxHeight->setDecimals(4);

        gridLayout_3->addWidget(doubleSpinBoxHeight, 2, 1, 1, 1);

        horizontalSliderHeight = new QSlider(groupBox_6);
        horizontalSliderHeight->setObjectName(QString::fromUtf8("horizontalSliderHeight"));
        horizontalSliderHeight->setMaximum(5000);
        horizontalSliderHeight->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(horizontalSliderHeight, 2, 2, 1, 1);


        verticalLayout_8->addLayout(gridLayout_3);


        verticalLayout_7->addWidget(groupBox_6);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_7->addItem(verticalSpacer);

        tabWidget_5->addTab(tab_11, QString());

        verticalLayout_11->addWidget(tabWidget_5);

        tabWidget_2->addTab(tabCost, QString());
        tabRobot = new QWidget();
        tabRobot->setObjectName(QString::fromUtf8("tabRobot"));
        verticalLayout_12 = new QVBoxLayout(tabRobot);
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setContentsMargins(11, 11, 11, 11);
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        verticalLayout_12->setContentsMargins(-1, 0, -1, -1);
        tabWidget_6 = new QTabWidget(tabRobot);
        tabWidget_6->setObjectName(QString::fromUtf8("tabWidget_6"));
        tabWidget_6->setUsesScrollButtons(true);
        tabWidget_6->setDocumentMode(true);
        tab_7 = new QWidget();
        tab_7->setObjectName(QString::fromUtf8("tab_7"));
        verticalLayout_13 = new QVBoxLayout(tab_7);
        verticalLayout_13->setSpacing(6);
        verticalLayout_13->setContentsMargins(11, 11, 11, 11);
        verticalLayout_13->setObjectName(QString::fromUtf8("verticalLayout_13"));
        widget = new MoveRobot(tab_7);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setMaximumSize(QSize(500, 16777215));

        verticalLayout_13->addWidget(widget);

        tabWidget_6->addTab(tab_7, QString());
        tab_13 = new QWidget();
        tab_13->setObjectName(QString::fromUtf8("tab_13"));
        verticalLayout_14 = new QVBoxLayout(tab_13);
        verticalLayout_14->setSpacing(6);
        verticalLayout_14->setContentsMargins(11, 11, 11, 11);
        verticalLayout_14->setObjectName(QString::fromUtf8("verticalLayout_14"));
        groupBox_11 = new QGroupBox(tab_13);
        groupBox_11->setObjectName(QString::fromUtf8("groupBox_11"));
        verticalLayout_15 = new QVBoxLayout(groupBox_11);
        verticalLayout_15->setSpacing(6);
        verticalLayout_15->setContentsMargins(11, 11, 11, 11);
        verticalLayout_15->setObjectName(QString::fromUtf8("verticalLayout_15"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_7);

        pushButtonTestAll = new QPushButton(groupBox_11);
        pushButtonTestAll->setObjectName(QString::fromUtf8("pushButtonTestAll"));

        horizontalLayout_2->addWidget(pushButtonTestAll);

        horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_8);


        verticalLayout_15->addLayout(horizontalLayout_2);

        line = new QFrame(groupBox_11);
        line->setObjectName(QString::fromUtf8("line"));
        line->setMinimumSize(QSize(300, 0));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_15->addWidget(line);

        gridLayout_5 = new QGridLayout();
        gridLayout_5->setSpacing(6);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        pushButtonCollision = new QPushButton(groupBox_11);
        pushButtonCollision->setObjectName(QString::fromUtf8("pushButtonCollision"));

        gridLayout_5->addWidget(pushButtonCollision, 0, 1, 1, 1);

        pushButtonLocalPath = new QPushButton(groupBox_11);
        pushButtonLocalPath->setObjectName(QString::fromUtf8("pushButtonLocalPath"));

        gridLayout_5->addWidget(pushButtonLocalPath, 1, 1, 1, 1);

        pushButtonCost = new QPushButton(groupBox_11);
        pushButtonCost->setObjectName(QString::fromUtf8("pushButtonCost"));

        gridLayout_5->addWidget(pushButtonCost, 2, 1, 1, 1);

        labelCollision = new QLabel(groupBox_11);
        labelCollision->setObjectName(QString::fromUtf8("labelCollision"));

        gridLayout_5->addWidget(labelCollision, 0, 2, 1, 1);

        labelLocalPath = new QLabel(groupBox_11);
        labelLocalPath->setObjectName(QString::fromUtf8("labelLocalPath"));

        gridLayout_5->addWidget(labelLocalPath, 1, 2, 1, 1);

        labelTimeCost = new QLabel(groupBox_11);
        labelTimeCost->setObjectName(QString::fromUtf8("labelTimeCost"));

        gridLayout_5->addWidget(labelTimeCost, 2, 2, 1, 1);

        horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_5->addItem(horizontalSpacer_10, 1, 3, 1, 1);

        horizontalSpacer_9 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_5->addItem(horizontalSpacer_9, 1, 0, 1, 1);


        verticalLayout_15->addLayout(gridLayout_5);


        verticalLayout_14->addWidget(groupBox_11);

        groupBox_12 = new QGroupBox(tab_13);
        groupBox_12->setObjectName(QString::fromUtf8("groupBox_12"));
        verticalLayout_16 = new QVBoxLayout(groupBox_12);
        verticalLayout_16->setSpacing(6);
        verticalLayout_16->setContentsMargins(11, 11, 11, 11);
        verticalLayout_16->setObjectName(QString::fromUtf8("verticalLayout_16"));
        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        horizontalSpacer_11 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_11->addItem(horizontalSpacer_11);

        pushButtonAttMat = new QPushButton(groupBox_12);
        pushButtonAttMat->setObjectName(QString::fromUtf8("pushButtonAttMat"));

        horizontalLayout_11->addWidget(pushButtonAttMat);

        horizontalSpacer_12 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_11->addItem(horizontalSpacer_12);


        verticalLayout_16->addLayout(horizontalLayout_11);


        verticalLayout_14->addWidget(groupBox_12);

        groupBoxGrabObject = new QGroupBox(tab_13);
        groupBoxGrabObject->setObjectName(QString::fromUtf8("groupBoxGrabObject"));
        gridLayout_15 = new QGridLayout(groupBoxGrabObject);
        gridLayout_15->setSpacing(6);
        gridLayout_15->setContentsMargins(11, 11, 11, 11);
        gridLayout_15->setObjectName(QString::fromUtf8("gridLayout_15"));
        pushButtonGrabObject = new QPushButton(groupBoxGrabObject);
        pushButtonGrabObject->setObjectName(QString::fromUtf8("pushButtonGrabObject"));

        gridLayout_15->addWidget(pushButtonGrabObject, 0, 0, 1, 1);

        comboBoxGrabObject = new QComboBox(groupBoxGrabObject);
        comboBoxGrabObject->setObjectName(QString::fromUtf8("comboBoxGrabObject"));

        gridLayout_15->addWidget(comboBoxGrabObject, 0, 1, 1, 1);

        pushButtonReleaseObject = new QPushButton(groupBoxGrabObject);
        pushButtonReleaseObject->setObjectName(QString::fromUtf8("pushButtonReleaseObject"));

        gridLayout_15->addWidget(pushButtonReleaseObject, 1, 0, 1, 1);


        verticalLayout_14->addWidget(groupBoxGrabObject);

        verticalSpacer_5 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_14->addItem(verticalSpacer_5);

        tabWidget_6->addTab(tab_13, QString());

        verticalLayout_12->addWidget(tabWidget_6);

        tabWidget_2->addTab(tabRobot, QString());
        tabUtils = new QWidget();
        tabUtils->setObjectName(QString::fromUtf8("tabUtils"));
        verticalLayout_17 = new QVBoxLayout(tabUtils);
        verticalLayout_17->setSpacing(6);
        verticalLayout_17->setContentsMargins(11, 11, 11, 11);
        verticalLayout_17->setObjectName(QString::fromUtf8("verticalLayout_17"));
        verticalLayout_17->setContentsMargins(-1, 0, -1, -1);
        tabWidget_3 = new QTabWidget(tabUtils);
        tabWidget_3->setObjectName(QString::fromUtf8("tabWidget_3"));
        tabWidget_3->setDocumentMode(true);
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        verticalLayout_18 = new QVBoxLayout(tab_3);
        verticalLayout_18->setSpacing(6);
        verticalLayout_18->setContentsMargins(11, 11, 11, 11);
        verticalLayout_18->setObjectName(QString::fromUtf8("verticalLayout_18"));
        gridLayout_6 = new QGridLayout();
        gridLayout_6->setSpacing(6);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        checkBoxDebug = new QCheckBox(tab_3);
        checkBoxDebug->setObjectName(QString::fromUtf8("checkBoxDebug"));

        gridLayout_6->addWidget(checkBoxDebug, 0, 1, 1, 1);

        checkBoxRecomputeTrajCost = new QCheckBox(tab_3);
        checkBoxRecomputeTrajCost->setObjectName(QString::fromUtf8("checkBoxRecomputeTrajCost"));

        gridLayout_6->addWidget(checkBoxRecomputeTrajCost, 1, 1, 1, 1);

        checkBoxWithShortCut_2 = new QCheckBox(tab_3);
        checkBoxWithShortCut_2->setObjectName(QString::fromUtf8("checkBoxWithShortCut_2"));

        gridLayout_6->addWidget(checkBoxWithShortCut_2, 0, 2, 1, 1);

        checkBoxUseTRRT = new QCheckBox(tab_3);
        checkBoxUseTRRT->setObjectName(QString::fromUtf8("checkBoxUseTRRT"));

        gridLayout_6->addWidget(checkBoxUseTRRT, 1, 2, 1, 1);

        horizontalSpacer_13 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_6->addItem(horizontalSpacer_13, 1, 0, 1, 1);

        horizontalSpacer_14 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_6->addItem(horizontalSpacer_14, 1, 3, 1, 1);


        verticalLayout_18->addLayout(gridLayout_6);

        greedyLayout = new QVBoxLayout();
        greedyLayout->setSpacing(6);
        greedyLayout->setObjectName(QString::fromUtf8("greedyLayout"));

        verticalLayout_18->addLayout(greedyLayout);

        verticalSpacer_8 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_18->addItem(verticalSpacer_8);

        tabWidget_3->addTab(tab_3, QString());

        verticalLayout_17->addWidget(tabWidget_3);

        tabWidget_2->addTab(tabUtils, QString());
        tabViewerSettings = new QWidget();
        tabViewerSettings->setObjectName(QString::fromUtf8("tabViewerSettings"));
        verticalLayout_31 = new QVBoxLayout(tabViewerSettings);
        verticalLayout_31->setSpacing(6);
        verticalLayout_31->setContentsMargins(11, 11, 11, 11);
        verticalLayout_31->setObjectName(QString::fromUtf8("verticalLayout_31"));
        horizontalLayout_22 = new QHBoxLayout();
        horizontalLayout_22->setSpacing(6);
        horizontalLayout_22->setObjectName(QString::fromUtf8("horizontalLayout_22"));
        groupBox_5 = new QGroupBox(tabViewerSettings);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        gridLayout_11 = new QGridLayout(groupBox_5);
        gridLayout_11->setSpacing(6);
        gridLayout_11->setContentsMargins(11, 11, 11, 11);
        gridLayout_11->setObjectName(QString::fromUtf8("gridLayout_11"));
        checkBoxTiles = new QCheckBox(groupBox_5);
        checkBoxTiles->setObjectName(QString::fromUtf8("checkBoxTiles"));

        gridLayout_11->addWidget(checkBoxTiles, 0, 0, 1, 1);

        checkBoxWalls = new QCheckBox(groupBox_5);
        checkBoxWalls->setObjectName(QString::fromUtf8("checkBoxWalls"));

        gridLayout_11->addWidget(checkBoxWalls, 0, 1, 1, 1);

        checkBoxSmooth = new QCheckBox(groupBox_5);
        checkBoxSmooth->setObjectName(QString::fromUtf8("checkBoxSmooth"));

        gridLayout_11->addWidget(checkBoxSmooth, 0, 2, 1, 1);

        checkBoxFloor = new QCheckBox(groupBox_5);
        checkBoxFloor->setObjectName(QString::fromUtf8("checkBoxFloor"));

        gridLayout_11->addWidget(checkBoxFloor, 1, 0, 1, 1);

        checkBoxShadows = new QCheckBox(groupBox_5);
        checkBoxShadows->setObjectName(QString::fromUtf8("checkBoxShadows"));

        gridLayout_11->addWidget(checkBoxShadows, 1, 1, 1, 1);

        checkBoxAxis = new QCheckBox(groupBox_5);
        checkBoxAxis->setObjectName(QString::fromUtf8("checkBoxAxis"));

        gridLayout_11->addWidget(checkBoxAxis, 1, 2, 1, 1);


        horizontalLayout_22->addWidget(groupBox_5);

        groupBox_4 = new QGroupBox(tabViewerSettings);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        gridLayout_12 = new QGridLayout(groupBox_4);
        gridLayout_12->setSpacing(6);
        gridLayout_12->setContentsMargins(11, 11, 11, 11);
        gridLayout_12->setObjectName(QString::fromUtf8("gridLayout_12"));
        checkBoxDrawGraph = new QCheckBox(groupBox_4);
        checkBoxDrawGraph->setObjectName(QString::fromUtf8("checkBoxDrawGraph"));

        gridLayout_12->addWidget(checkBoxDrawGraph, 0, 0, 1, 1);

        checkBoxDrawTraj = new QCheckBox(groupBox_4);
        checkBoxDrawTraj->setObjectName(QString::fromUtf8("checkBoxDrawTraj"));

        gridLayout_12->addWidget(checkBoxDrawTraj, 1, 0, 1, 1);


        horizontalLayout_22->addWidget(groupBox_4);


        verticalLayout_31->addLayout(horizontalLayout_22);

        groupBox_3 = new QGroupBox(tabViewerSettings);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        gridLayout_13 = new QGridLayout(groupBox_3);
        gridLayout_13->setSpacing(6);
        gridLayout_13->setContentsMargins(11, 11, 11, 11);
        gridLayout_13->setObjectName(QString::fromUtf8("gridLayout_13"));
        checkBoxBB = new QCheckBox(groupBox_3);
        checkBoxBB->setObjectName(QString::fromUtf8("checkBoxBB"));

        gridLayout_13->addWidget(checkBoxBB, 0, 0, 1, 1);

        checkBoxGhosts = new QCheckBox(groupBox_3);
        checkBoxGhosts->setObjectName(QString::fromUtf8("checkBoxGhosts"));

        gridLayout_13->addWidget(checkBoxGhosts, 1, 0, 1, 1);

        checkBoxFilaire = new QCheckBox(groupBox_3);
        checkBoxFilaire->setObjectName(QString::fromUtf8("checkBoxFilaire"));

        gridLayout_13->addWidget(checkBoxFilaire, 1, 1, 1, 1);


        verticalLayout_31->addWidget(groupBox_3);

        groupBox_13 = new QGroupBox(tabViewerSettings);
        groupBox_13->setObjectName(QString::fromUtf8("groupBox_13"));
        gridLayout_21 = new QGridLayout(groupBox_13);
        gridLayout_21->setSpacing(6);
        gridLayout_21->setContentsMargins(11, 11, 11, 11);
        gridLayout_21->setObjectName(QString::fromUtf8("gridLayout_21"));
        pushButtonRestoreLight = new QPushButton(groupBox_13);
        pushButtonRestoreLight->setObjectName(QString::fromUtf8("pushButtonRestoreLight"));

        gridLayout_21->addWidget(pushButtonRestoreLight, 0, 4, 1, 1);

        doubleSpinBoxLightX = new QDoubleSpinBox(groupBox_13);
        doubleSpinBoxLightX->setObjectName(QString::fromUtf8("doubleSpinBoxLightX"));

        gridLayout_21->addWidget(doubleSpinBoxLightX, 0, 0, 1, 1);

        label_2 = new QLabel(groupBox_13);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_21->addWidget(label_2, 0, 1, 1, 1);

        horizontalSliderLightX = new QSlider(groupBox_13);
        horizontalSliderLightX->setObjectName(QString::fromUtf8("horizontalSliderLightX"));
        horizontalSliderLightX->setMaximum(1000);
        horizontalSliderLightX->setOrientation(Qt::Horizontal);

        gridLayout_21->addWidget(horizontalSliderLightX, 0, 3, 1, 1);

        horizontalSliderLightY = new QSlider(groupBox_13);
        horizontalSliderLightY->setObjectName(QString::fromUtf8("horizontalSliderLightY"));
        horizontalSliderLightY->setMaximum(1000);
        horizontalSliderLightY->setOrientation(Qt::Horizontal);

        gridLayout_21->addWidget(horizontalSliderLightY, 1, 3, 1, 1);

        doubleSpinBoxLightY = new QDoubleSpinBox(groupBox_13);
        doubleSpinBoxLightY->setObjectName(QString::fromUtf8("doubleSpinBoxLightY"));

        gridLayout_21->addWidget(doubleSpinBoxLightY, 1, 0, 1, 1);

        label_3 = new QLabel(groupBox_13);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_21->addWidget(label_3, 1, 1, 1, 1);

        doubleSpinBoxLightZ = new QDoubleSpinBox(groupBox_13);
        doubleSpinBoxLightZ->setObjectName(QString::fromUtf8("doubleSpinBoxLightZ"));

        gridLayout_21->addWidget(doubleSpinBoxLightZ, 2, 0, 1, 1);

        label_4 = new QLabel(groupBox_13);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_21->addWidget(label_4, 2, 1, 1, 1);

        horizontalSliderLightZ = new QSlider(groupBox_13);
        horizontalSliderLightZ->setObjectName(QString::fromUtf8("horizontalSliderLightZ"));
        horizontalSliderLightZ->setMaximum(1000);
        horizontalSliderLightZ->setOrientation(Qt::Horizontal);

        gridLayout_21->addWidget(horizontalSliderLightZ, 2, 3, 1, 1);

        checkBoxDrawLightSource = new QCheckBox(groupBox_13);
        checkBoxDrawLightSource->setObjectName(QString::fromUtf8("checkBoxDrawLightSource"));

        gridLayout_21->addWidget(checkBoxDrawLightSource, 1, 4, 1, 1);


        verticalLayout_31->addWidget(groupBox_13);

        widget_8 = new QWidget(tabViewerSettings);
        widget_8->setObjectName(QString::fromUtf8("widget_8"));
        gridLayout_20 = new QGridLayout(widget_8);
        gridLayout_20->setSpacing(6);
        gridLayout_20->setContentsMargins(11, 11, 11, 11);
        gridLayout_20->setObjectName(QString::fromUtf8("gridLayout_20"));
        horizontalSpacer_18 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_20->addItem(horizontalSpacer_18, 1, 0, 1, 1);

        pushButtonRestoreView = new QPushButton(widget_8);
        pushButtonRestoreView->setObjectName(QString::fromUtf8("pushButtonRestoreView"));

        gridLayout_20->addWidget(pushButtonRestoreView, 1, 1, 1, 1);


        verticalLayout_31->addWidget(widget_8);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_31->addItem(verticalSpacer_4);

        tabWidget_2->addTab(tabViewerSettings, QString());

        verticalLayout_6->addWidget(tabWidget_2);

        splitter->addWidget(sidePanel);
        verticalLayout = new QWidget(splitter);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(3);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(verticalLayout->sizePolicy().hasHeightForWidth());
        verticalLayout->setSizePolicy(sizePolicy3);
        verticalLayout_2 = new QVBoxLayout(verticalLayout);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(-1, -1, -1, 2);
        splitter_2 = new QSplitter(verticalLayout);
        splitter_2->setObjectName(QString::fromUtf8("splitter_2"));
        splitter_2->setOrientation(Qt::Vertical);
        OpenGL = new GLWidget(splitter_2);
        OpenGL->setObjectName(QString::fromUtf8("OpenGL"));
        QSizePolicy sizePolicy4(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy4.setHorizontalStretch(2);
        sizePolicy4.setVerticalStretch(2);
        sizePolicy4.setHeightForWidth(OpenGL->sizePolicy().hasHeightForWidth());
        OpenGL->setSizePolicy(sizePolicy4);
        OpenGL->setMinimumSize(QSize(400, 300));
        splitter_2->addWidget(OpenGL);
        widget_3 = new QWidget(splitter_2);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        sizePolicy.setHeightForWidth(widget_3->sizePolicy().hasHeightForWidth());
        widget_3->setSizePolicy(sizePolicy);
        horizontalLayout_3 = new QHBoxLayout(widget_3);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(-1, -1, -1, 2);
        groupBox_20 = new QGroupBox(widget_3);
        groupBox_20->setObjectName(QString::fromUtf8("groupBox_20"));
        verticalLayout_32 = new QVBoxLayout(groupBox_20);
        verticalLayout_32->setSpacing(6);
        verticalLayout_32->setContentsMargins(11, 11, 11, 11);
        verticalLayout_32->setObjectName(QString::fromUtf8("verticalLayout_32"));
        widget_4 = new QWidget(groupBox_20);
        widget_4->setObjectName(QString::fromUtf8("widget_4"));
        QSizePolicy sizePolicy5(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(widget_4->sizePolicy().hasHeightForWidth());
        widget_4->setSizePolicy(sizePolicy5);
        widget_4->setMinimumSize(QSize(319, 45));
        pushButtonReset = new QPushButton(widget_4);
        pushButtonReset->setObjectName(QString::fromUtf8("pushButtonReset"));
        pushButtonReset->setGeometry(QRect(100, 0, 40, 40));
        sizePolicy5.setHeightForWidth(pushButtonReset->sizePolicy().hasHeightForWidth());
        pushButtonReset->setSizePolicy(sizePolicy5);
        pushButtonReset->setMinimumSize(QSize(0, 0));
        pushButtonReset->setMaximumSize(QSize(16777215, 16777215));
        pushButtonReset->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-stop.svg) 1 1 1 1 ;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-stop.svg) -2 -2 -2 -2 ;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-stop.svg) -5 -5 -5 -5 ;\n"
"}\n"
"\n"
"QPushButton:disabled {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-stop-disabled.svg) 1 1 1 1 ;\n"
"}"));
        pushButtonResetGraph = new QPushButton(widget_4);
        pushButtonResetGraph->setObjectName(QString::fromUtf8("pushButtonResetGraph"));
        pushButtonResetGraph->setGeometry(QRect(200, 10, 117, 32));
        QSizePolicy sizePolicy6(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy6.setHorizontalStretch(0);
        sizePolicy6.setVerticalStretch(0);
        sizePolicy6.setHeightForWidth(pushButtonResetGraph->sizePolicy().hasHeightForWidth());
        pushButtonResetGraph->setSizePolicy(sizePolicy6);
        pushButtonRun = new QPushButton(widget_4);
        pushButtonRun->setObjectName(QString::fromUtf8("pushButtonRun"));
        pushButtonRun->setGeometry(QRect(0, 0, 40, 40));
        sizePolicy5.setHeightForWidth(pushButtonRun->sizePolicy().hasHeightForWidth());
        pushButtonRun->setSizePolicy(sizePolicy5);
        pushButtonRun->setMinimumSize(QSize(0, 0));
        pushButtonRun->setMaximumSize(QSize(16777215, 16777215));
        pushButtonRun->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-start.svg) 1 1 1 1 ;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-start.svg) -2 -2 -2 -2 ;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-start.svg) -8 -8 -8 -8 ;\n"
"}\n"
"\n"
"QPushButton:disabled {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-start-disabled.svg) 1 1 1 1 ;\n"
"}"));
        pushButtonRun->setIconSize(QSize(50, 50));
        pushButtonStop = new QPushButton(widget_4);
        pushButtonStop->setObjectName(QString::fromUtf8("pushButtonStop"));
        pushButtonStop->setGeometry(QRect(50, 0, 40, 40));
        sizePolicy5.setHeightForWidth(pushButtonStop->sizePolicy().hasHeightForWidth());
        pushButtonStop->setSizePolicy(sizePolicy5);
        pushButtonStop->setMinimumSize(QSize(0, 0));
        pushButtonStop->setMaximumSize(QSize(16777215, 16777215));
        pushButtonStop->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-pause.svg) 1 1 1 1 ;\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-pause.svg) -2 -2 -2 -2 ;\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-pause.svg) -5 -5 -5 -5 ;\n"
"}\n"
"\n"
"QPushButton:disabled {\n"
"border-image:url(/Users/jmainpri/workspace/BioMove3D/qtWindow/images/media-playback-pause-disabled.svg) 1 1 1 1 ;\n"
"}"));
        pushButtonStop->setIconSize(QSize(50, 50));

        verticalLayout_32->addWidget(widget_4);

        horizontalLayout_24 = new QHBoxLayout();
        horizontalLayout_24->setSpacing(6);
        horizontalLayout_24->setObjectName(QString::fromUtf8("horizontalLayout_24"));
        radioButtonDiff = new QRadioButton(groupBox_20);
        radioButtonDiff->setObjectName(QString::fromUtf8("radioButtonDiff"));

        horizontalLayout_24->addWidget(radioButtonDiff);

        radioButtonPRM = new QRadioButton(groupBox_20);
        radioButtonPRM->setObjectName(QString::fromUtf8("radioButtonPRM"));

        horizontalLayout_24->addWidget(radioButtonPRM);

        checkBoxWithShortCut = new QCheckBox(groupBox_20);
        checkBoxWithShortCut->setObjectName(QString::fromUtf8("checkBoxWithShortCut"));

        horizontalLayout_24->addWidget(checkBoxWithShortCut);


        verticalLayout_32->addLayout(horizontalLayout_24);


        horizontalLayout_3->addWidget(groupBox_20);

        groupBox_19 = new QGroupBox(widget_3);
        groupBox_19->setObjectName(QString::fromUtf8("groupBox_19"));
        verticalLayout_34 = new QVBoxLayout(groupBox_19);
        verticalLayout_34->setSpacing(6);
        verticalLayout_34->setContentsMargins(11, 11, 11, 11);
        verticalLayout_34->setObjectName(QString::fromUtf8("verticalLayout_34"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalSpacer_19 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_19);

        pushButtonShowTraj = new QPushButton(groupBox_19);
        pushButtonShowTraj->setObjectName(QString::fromUtf8("pushButtonShowTraj"));

        horizontalLayout_4->addWidget(pushButtonShowTraj);

        horizontalSpacer_20 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_20);


        verticalLayout_34->addLayout(horizontalLayout_4);

        horizontalLayout_25 = new QHBoxLayout();
        horizontalLayout_25->setSpacing(6);
        horizontalLayout_25->setObjectName(QString::fromUtf8("horizontalLayout_25"));
        label = new QLabel(groupBox_19);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_25->addWidget(label);

        doubleSpinBox = new QDoubleSpinBox(groupBox_19);
        doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));

        horizontalLayout_25->addWidget(doubleSpinBox);

        horizontalSlider = new QSlider(groupBox_19);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        horizontalLayout_25->addWidget(horizontalSlider);


        verticalLayout_34->addLayout(horizontalLayout_25);


        horizontalLayout_3->addWidget(groupBox_19);

        splitter_2->addWidget(widget_3);

        verticalLayout_2->addWidget(splitter_2);

        splitter->addWidget(verticalLayout);

        verticalLayout_29->addWidget(splitter);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1482, 22));
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

        tabWidget_2->setCurrentIndex(0);
        tabWidget_4->setCurrentIndex(0);
        tabWidget_5->setCurrentIndex(1);
        tabWidget_6->setCurrentIndex(0);
        tabWidget_3->setCurrentIndex(0);


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
        isWithGoal->setText(QApplication::translate("MainWindow", "With Goal", 0, QApplication::UnicodeUTF8));
        isBidir->setText(QApplication::translate("MainWindow", "Bidirectional", 0, QApplication::UnicodeUTF8));
        isBalanced->setText(QApplication::translate("MainWindow", "Balanced", 0, QApplication::UnicodeUTF8));
        isExpandControl->setText(QApplication::translate("MainWindow", "Expand Control", 0, QApplication::UnicodeUTF8));
        isManhattan->setText(QApplication::translate("MainWindow", "Manhatan", 0, QApplication::UnicodeUTF8));
        isDiscardingNodes->setText(QApplication::translate("MainWindow", "Discard Nodes", 0, QApplication::UnicodeUTF8));
        isCostSpace->setText(QApplication::translate("MainWindow", "Cost Space", 0, QApplication::UnicodeUTF8));
        isCostTransition->setText(QApplication::translate("MainWindow", "Cost T-Before", 0, QApplication::UnicodeUTF8));
        isEST->setText(QApplication::translate("MainWindow", "EST Planner", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Tree Expansion Method", 0, QApplication::UnicodeUTF8));
        expansionMethod->clear();
        expansionMethod->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "Extend", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Extend n Step", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Connect", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Cost Connect", 0, QApplication::UnicodeUTF8)
        );
        isManualRefiRadius->setText(QApplication::translate("MainWindow", "Manula refi. radius", 0, QApplication::UnicodeUTF8));
        isAddingCycles->setText(QApplication::translate("MainWindow", "Add Cycles", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "Step (Delta)", 0, QApplication::UnicodeUTF8));
        checkBoxIsGoalBias->setText(QApplication::translate("MainWindow", "Goal Bias", 0, QApplication::UnicodeUTF8));
        labelMaxNodes->setText(QApplication::translate("MainWindow", "Max Node", 0, QApplication::UnicodeUTF8));
        tabWidget_4->setTabText(tabWidget_4->indexOf(tabDiffu), QApplication::translate("MainWindow", "Diffusion", 0, QApplication::UnicodeUTF8));
        groupBox_17->setTitle(QApplication::translate("MainWindow", "Planning strategy", 0, QApplication::UnicodeUTF8));
        comboBox_2->clear();
        comboBox_2->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "PRM", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Visibility PRM", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "ACR", 0, QApplication::UnicodeUTF8)
        );
        checkBox_9->setText(QApplication::translate("MainWindow", "Oriented", 0, QApplication::UnicodeUTF8));
        groupBox_18->setTitle(QApplication::translate("MainWindow", "Connection strategy", 0, QApplication::UnicodeUTF8));
        comboBox_3->clear();
        comboBox_3->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "Nb Edges", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Distance", 0, QApplication::UnicodeUTF8)
        );
        groupBox_16->setTitle(QApplication::translate("MainWindow", "Sampling strategy", 0, QApplication::UnicodeUTF8));
        radioButton->setText(QApplication::translate("MainWindow", "Random", 0, QApplication::UnicodeUTF8));
        radioButton_3->setText(QApplication::translate("MainWindow", "Halton", 0, QApplication::UnicodeUTF8));
        radioButton_2->setText(QApplication::translate("MainWindow", "Gaussian", 0, QApplication::UnicodeUTF8));
        radioButton_4->setText(QApplication::translate("MainWindow", "Bridge", 0, QApplication::UnicodeUTF8));
        radioButton_5->setText(QApplication::translate("MainWindow", "OBPRM", 0, QApplication::UnicodeUTF8));
        tabWidget_4->setTabText(tabWidget_4->indexOf(tabPRM), QApplication::translate("MainWindow", "PRM", 0, QApplication::UnicodeUTF8));
        tabWidget_4->setTabText(tabWidget_4->indexOf(tabOpti), QApplication::translate("MainWindow", "Optimization", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("MainWindow", "New Context Name", 0, QApplication::UnicodeUTF8));
        pushButtonSaveContext->setText(QApplication::translate("MainWindow", "Add", 0, QApplication::UnicodeUTF8));
        pushButtonResetContext->setText(QApplication::translate("MainWindow", "Reset Stack", 0, QApplication::UnicodeUTF8));
        pushButtonPrintAllContext->setText(QApplication::translate("MainWindow", "Print Stack", 0, QApplication::UnicodeUTF8));
        pushButtonPrintSelected->setText(QApplication::translate("MainWindow", "Print Selected", 0, QApplication::UnicodeUTF8));
        pushButtonSetSelected->setText(QApplication::translate("MainWindow", "Set Selected", 0, QApplication::UnicodeUTF8));
        pushButtonRunAllRRT->setText(QApplication::translate("MainWindow", "Run Multi RRT", 0, QApplication::UnicodeUTF8));
        pushButtonRunAllGreedy->setText(QApplication::translate("MainWindow", "Run Multi Greedy", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("MainWindow", "Nb of Rounds", 0, QApplication::UnicodeUTF8));
        pushButtonShowHisto->setText(QApplication::translate("MainWindow", "Show Histo", 0, QApplication::UnicodeUTF8));
        tabWidget_4->setTabText(tabWidget_4->indexOf(tabMultiRun), QApplication::translate("MainWindow", "Multi-Run", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tabMotionPlanner), QApplication::translate("MainWindow", "Motion Planner", 0, QApplication::UnicodeUTF8));
        isCostSpaceCopy->setText(QApplication::translate("MainWindow", "Enable Cost Space", 0, QApplication::UnicodeUTF8));
        groupBox_7->setTitle(QApplication::translate("MainWindow", "General Cost Space", 0, QApplication::UnicodeUTF8));
        pushButtonShowTrajCost->setText(QApplication::translate("MainWindow", "Show Traj Cost", 0, QApplication::UnicodeUTF8));
        checkBoxRescale->setText(QApplication::translate("MainWindow", "Will not rescale", 0, QApplication::UnicodeUTF8));
        groupBox_10->setTitle(QApplication::translate("MainWindow", "T-RRT", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "Initial Temp.", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "NFailmax", 0, QApplication::UnicodeUTF8));
        checkBoxCostBefore->setText(QApplication::translate("MainWindow", "Compute Cost Before Collision", 0, QApplication::UnicodeUTF8));
        pushButtonShowTemp->setText(QApplication::translate("MainWindow", "Show Temp.", 0, QApplication::UnicodeUTF8));
        pushButtonGridInGraph->setText(QApplication::translate("MainWindow", "Grid In Graph", 0, QApplication::UnicodeUTF8));
        pushButtonAStar->setText(QApplication::translate("MainWindow", "Test A*", 0, QApplication::UnicodeUTF8));
        tabWidget_5->setTabText(tabWidget_5->indexOf(tab_12), QApplication::translate("MainWindow", "General", 0, QApplication::UnicodeUTF8));
        enableHri_2->setText(QApplication::translate("MainWindow", "Enable Human Robot Interactions", 0, QApplication::UnicodeUTF8));
        groupBox_9->setTitle(QApplication::translate("MainWindow", "Draw", 0, QApplication::UnicodeUTF8));
        checkBoxDrawDistance->setText(QApplication::translate("MainWindow", "Draw Distance", 0, QApplication::UnicodeUTF8));
        checkBoxDrawGrid->setText(QApplication::translate("MainWindow", "Draw Grid", 0, QApplication::UnicodeUTF8));
        checkBoxDrawRandPoints->setText(QApplication::translate("MainWindow", "Draw RandPts", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindow", "Joint To Assign Cost", 0, QApplication::UnicodeUTF8));
        pushButtonHRITS->setText(QApplication::translate("MainWindow", "enable TS HRI", 0, QApplication::UnicodeUTF8));
        HRITaskSpace->setTitle(QApplication::translate("MainWindow", "Hri Task Space", 0, QApplication::UnicodeUTF8));
        enableHriTS->setText(QApplication::translate("MainWindow", "HRICS TS", 0, QApplication::UnicodeUTF8));
        pushButtonWorkspacePath->setText(QApplication::translate("MainWindow", "Workspace A*", 0, QApplication::UnicodeUTF8));
        pushButtonHoleMotion->setText(QApplication::translate("MainWindow", "Entire Path", 0, QApplication::UnicodeUTF8));
        groupBox_8->setTitle(QApplication::translate("MainWindow", "My Grid", 0, QApplication::UnicodeUTF8));
        pushButtonMakeGrid->setText(QApplication::translate("MainWindow", "New HRICS Planner", 0, QApplication::UnicodeUTF8));
        pushButtonDeleteGrid->setText(QApplication::translate("MainWindow", "Delete Planner", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindow", "Cell Size", 0, QApplication::UnicodeUTF8));
        HRICSPlanner->setTitle(QApplication::translate("MainWindow", "HRICS Planner", 0, QApplication::UnicodeUTF8));
        checkBoxHRICS_MOPL->setText(QApplication::translate("MainWindow", "HRICS WS", 0, QApplication::UnicodeUTF8));
        pushButtonComputeCost->setText(QApplication::translate("MainWindow", "Compute Cost", 0, QApplication::UnicodeUTF8));
        pushButtonResetCost->setText(QApplication::translate("MainWindow", "Reset Cost", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindow", "Zone Size", 0, QApplication::UnicodeUTF8));
        pushButtonHRICSRRT->setText(QApplication::translate("MainWindow", "HRICS RRT", 0, QApplication::UnicodeUTF8));
        pushButtonAStaIn3DGrid->setText(QApplication::translate("MainWindow", "HRICS A* ", 0, QApplication::UnicodeUTF8));
        checkBoxInverseKinematics->setText(QApplication::translate("MainWindow", "Inverse Kinematics", 0, QApplication::UnicodeUTF8));
        pushButtonResetRandPoints->setText(QApplication::translate("MainWindow", "Reset RandPts", 0, QApplication::UnicodeUTF8));
        checkBoxHRIGoalBiased->setText(QApplication::translate("MainWindow", "Goal Biased", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "Hri CSpace", 0, QApplication::UnicodeUTF8));
        checkBoxEnableHRIConfigSpace->setText(QApplication::translate("MainWindow", "HRICS CS", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("MainWindow", "Visibility", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("MainWindow", "Distance", 0, QApplication::UnicodeUTF8));
        whichTestBox->clear();
        whichTestBox->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "Distance", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Visibility", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "Combined", 0, QApplication::UnicodeUTF8)
        );
        pushButtonCreateGrid->setText(QApplication::translate("MainWindow", "Create Grid", 0, QApplication::UnicodeUTF8));
        pushButtonNewHRICSpace->setText(QApplication::translate("MainWindow", "New", 0, QApplication::UnicodeUTF8));
        pushButtonDeleteHRICSpace->setText(QApplication::translate("MainWindow", "Delete", 0, QApplication::UnicodeUTF8));
        checkBoxBBDist->setText(QApplication::translate("MainWindow", "Box Dist", 0, QApplication::UnicodeUTF8));
        checkBoxBallDist->setText(QApplication::translate("MainWindow", "Ball Dist", 0, QApplication::UnicodeUTF8));
        tabWidget_5->setTabText(tabWidget_5->indexOf(tab_8), QApplication::translate("MainWindow", "Hri", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("MainWindow", "Global Coef", 0, QApplication::UnicodeUTF8));
        groupBox_6->setTitle(QApplication::translate("MainWindow", "Natural Parameters", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("MainWindow", "Joint Limit", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("MainWindow", "Task Distance", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("MainWindow", "Height", 0, QApplication::UnicodeUTF8));
        tabWidget_5->setTabText(tabWidget_5->indexOf(tab_11), QApplication::translate("MainWindow", "Human-Like", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tabCost), QApplication::translate("MainWindow", "Cost", 0, QApplication::UnicodeUTF8));
        tabWidget_6->setTabText(tabWidget_6->indexOf(tab_7), QApplication::translate("MainWindow", "Robot Pos", 0, QApplication::UnicodeUTF8));
        groupBox_11->setTitle(QApplication::translate("MainWindow", "Time", 0, QApplication::UnicodeUTF8));
        pushButtonTestAll->setText(QApplication::translate("MainWindow", "Test All", 0, QApplication::UnicodeUTF8));
        pushButtonCollision->setText(QApplication::translate("MainWindow", "Collision", 0, QApplication::UnicodeUTF8));
        pushButtonLocalPath->setText(QApplication::translate("MainWindow", "LocalPath", 0, QApplication::UnicodeUTF8));
        pushButtonCost->setText(QApplication::translate("MainWindow", "Cost", 0, QApplication::UnicodeUTF8));
        labelCollision->setText(QApplication::translate("MainWindow", "Collision Time", 0, QApplication::UnicodeUTF8));
        labelLocalPath->setText(QApplication::translate("MainWindow", "Local Path Time", 0, QApplication::UnicodeUTF8));
        labelTimeCost->setText(QApplication::translate("MainWindow", "Cost Time", 0, QApplication::UnicodeUTF8));
        groupBox_12->setTitle(QApplication::translate("MainWindow", "Inverse Kinematiks", 0, QApplication::UnicodeUTF8));
        pushButtonAttMat->setText(QApplication::translate("MainWindow", "Set Att Matrix", 0, QApplication::UnicodeUTF8));
        groupBoxGrabObject->setTitle(QApplication::translate("MainWindow", "Grab Object", 0, QApplication::UnicodeUTF8));
        pushButtonGrabObject->setText(QApplication::translate("MainWindow", "Grab Object", 0, QApplication::UnicodeUTF8));
        comboBoxGrabObject->clear();
        comboBoxGrabObject->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "Aucun Objet", 0, QApplication::UnicodeUTF8)
        );
        pushButtonReleaseObject->setText(QApplication::translate("MainWindow", "Release Object", 0, QApplication::UnicodeUTF8));
        tabWidget_6->setTabText(tabWidget_6->indexOf(tab_13), QApplication::translate("MainWindow", "Model", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tabRobot), QApplication::translate("MainWindow", "Robot", 0, QApplication::UnicodeUTF8));
        checkBoxDebug->setText(QApplication::translate("MainWindow", "Debug", 0, QApplication::UnicodeUTF8));
        checkBoxRecomputeTrajCost->setText(QApplication::translate("MainWindow", "Recompute Traj Cost", 0, QApplication::UnicodeUTF8));
        checkBoxWithShortCut_2->setText(QApplication::translate("MainWindow", "With Short Cut", 0, QApplication::UnicodeUTF8));
        checkBoxUseTRRT->setText(QApplication::translate("MainWindow", "With T-RRT", 0, QApplication::UnicodeUTF8));
        tabWidget_3->setTabText(tabWidget_3->indexOf(tab_3), QApplication::translate("MainWindow", "Greedy", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tabUtils), QApplication::translate("MainWindow", "Utils", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("MainWindow", "Scene", 0, QApplication::UnicodeUTF8));
        checkBoxTiles->setText(QApplication::translate("MainWindow", "Tiles", 0, QApplication::UnicodeUTF8));
        checkBoxWalls->setText(QApplication::translate("MainWindow", "Walls", 0, QApplication::UnicodeUTF8));
        checkBoxSmooth->setText(QApplication::translate("MainWindow", "Smooth", 0, QApplication::UnicodeUTF8));
        checkBoxFloor->setText(QApplication::translate("MainWindow", "Floor", 0, QApplication::UnicodeUTF8));
        checkBoxShadows->setText(QApplication::translate("MainWindow", "Shadow", 0, QApplication::UnicodeUTF8));
        checkBoxAxis->setText(QApplication::translate("MainWindow", "Axis", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("MainWindow", "Motion Planning", 0, QApplication::UnicodeUTF8));
        checkBoxDrawGraph->setText(QApplication::translate("MainWindow", "Draw Graph", 0, QApplication::UnicodeUTF8));
        checkBoxDrawTraj->setText(QApplication::translate("MainWindow", "Draw Traj", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("MainWindow", "3D Model", 0, QApplication::UnicodeUTF8));
        checkBoxBB->setText(QApplication::translate("MainWindow", "Bounding Boxes", 0, QApplication::UnicodeUTF8));
        checkBoxGhosts->setText(QApplication::translate("MainWindow", "Ghosts", 0, QApplication::UnicodeUTF8));
        checkBoxFilaire->setText(QApplication::translate("MainWindow", "Filaire", 0, QApplication::UnicodeUTF8));
        groupBox_13->setTitle(QApplication::translate("MainWindow", "Light", 0, QApplication::UnicodeUTF8));
        pushButtonRestoreLight->setText(QApplication::translate("MainWindow", "Restore Light", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "X", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Y", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Z", 0, QApplication::UnicodeUTF8));
        checkBoxDrawLightSource->setText(QApplication::translate("MainWindow", "Draw Light Source", 0, QApplication::UnicodeUTF8));
        pushButtonRestoreView->setText(QApplication::translate("MainWindow", "Restore View", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tabViewerSettings), QApplication::translate("MainWindow", "Viewer Settings", 0, QApplication::UnicodeUTF8));
        groupBox_20->setTitle(QApplication::translate("MainWindow", "Run Motion Planning", 0, QApplication::UnicodeUTF8));
        pushButtonReset->setText(QString());
        pushButtonResetGraph->setText(QApplication::translate("MainWindow", "Reset Graph", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        pushButtonRun->setToolTip(QApplication::translate("MainWindow", "Run Motion Planner", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        pushButtonRun->setWhatsThis(QApplication::translate("MainWindow", "Run planning", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
        pushButtonRun->setText(QString());
#ifndef QT_NO_TOOLTIP
        pushButtonStop->setToolTip(QApplication::translate("MainWindow", "Run Motion Planner", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        pushButtonStop->setWhatsThis(QApplication::translate("MainWindow", "Run planning", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
        pushButtonStop->setText(QString());
        radioButtonDiff->setText(QApplication::translate("MainWindow", "Diffusion", 0, QApplication::UnicodeUTF8));
        radioButtonPRM->setText(QApplication::translate("MainWindow", "PRM", 0, QApplication::UnicodeUTF8));
        checkBoxWithShortCut->setText(QApplication::translate("MainWindow", "With Short Cut", 0, QApplication::UnicodeUTF8));
        groupBox_19->setTitle(QApplication::translate("MainWindow", "Trajectory", 0, QApplication::UnicodeUTF8));
        pushButtonShowTraj->setText(QApplication::translate("MainWindow", "Show Trajectory", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Speed", 0, QApplication::UnicodeUTF8));
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
