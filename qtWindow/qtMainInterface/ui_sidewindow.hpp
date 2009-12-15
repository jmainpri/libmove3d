/********************************************************************************
** Form generated from reading UI file 'sidewindow.ui'
**
** Created: Tue Dec 15 11:24:36 2009
**      by: Qt User Interface Compiler version 4.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef SIDEWINDOW_H
#define SIDEWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QToolBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "../qtFormRobot/moverobot.hpp"

QT_BEGIN_NAMESPACE

class Ui_SideWindow
{
public:
    QHBoxLayout *horizontalLayout;
    QTabWidget *Tabs;
    QWidget *MPlanning;
    QToolBox *toolBox;
    QWidget *PRMTab;
    QGroupBox *groupBox_2;
    QComboBox *comboBox_2;
    QCheckBox *checkBox_9;
    QGroupBox *groupBox_3;
    QComboBox *comboBox_3;
    QGroupBox *groupBox_4;
    QRadioButton *radioButton;
    QRadioButton *radioButton_2;
    QRadioButton *radioButton_3;
    QRadioButton *radioButton_4;
    QRadioButton *radioButton_5;
    QWidget *DiffTab;
    QGroupBox *groupBox;
    QCheckBox *isAddingCycles;
    QComboBox *expansionMethod;
    QCheckBox *isManualRefiRadius;
    QSlider *horizontalSliderExtentionStep;
    QDoubleSpinBox *doubleSpinBoxExtentionStep;
    QCheckBox *checkBox;
    QDoubleSpinBox *doubleSpinBoxBias;
    QSlider *horizontalSliderBias;
    QLabel *label_4;
    QCheckBox *isDiscardingNodes;
    QCheckBox *isBalanced;
    QCheckBox *isCostTransition;
    QCheckBox *isExpandControl;
    QCheckBox *isWithGoal;
    QCheckBox *isEST;
    QLabel *labelMaxNodes;
    QCheckBox *isCostSpace;
    QCheckBox *isManhattan;
    QCheckBox *isBidir;
    QLineEdit *lineEditMaxNodes;
    QWidget *Optimisation;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *optimizeLayout;
    QWidget *Cost;
    QGroupBox *groupBoxGeneralCostSpace;
    QPushButton *pushButtonShowTrajCost;
    QCheckBox *checkBoxRescale;
    QPushButton *pushButtonGridInGraph;
    QPushButton *pushButtonAStar;
    QCheckBox *isCostSpaceCopy;
    QGroupBox *groupBox_5;
    QSpinBox *spinBoxJoint;
    QLabel *label_8;
    QGroupBox *HRITaskSpace;
    QComboBox *whichTestBox;
    QCheckBox *enableHriTS;
    QPushButton *pushButtonWorkspacePath;
    QPushButton *pushButtonHoleMotion;
    QSlider *horizontalSliderVisibility;
    QDoubleSpinBox *doubleSpinBoxVisibility;
    QLabel *label_6;
    QPushButton *pushButtonHRITS;
    QCheckBox *enableHri;
    QGroupBox *groupBox_6;
    QDoubleSpinBox *doubleSpinBoxCellSize;
    QLabel *label;
    QPushButton *pushButtonMakeGrid;
    QPushButton *pushButtonDeleteGrid;
    QSlider *horizontalSliderCellSize;
    QGroupBox *HRICSPlanner;
    QCheckBox *checkBoxDrawGrid;
    QPushButton *pushButtonComputeCost;
    QPushButton *pushButtonAStaIn3DGrid;
    QPushButton *pushButtonResetCost;
    QLabel *label_3;
    QDoubleSpinBox *doubleSpinBoxZoneSize;
    QSlider *horizontalSliderZoneSize;
    QLabel *label_2;
    QDoubleSpinBox *doubleSpinBoxDistance;
    QSlider *horizontalSliderDistance;
    QCheckBox *checkBoxDrawDistance;
    QCheckBox *checkBoxHRICS_MOPL;
    QCheckBox *checkBoxBBDist;
    QPushButton *pushButtonHRICSRRT;
    QCheckBox *checkBoxCostBefore;
    QWidget *Greedy;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *greedyLayout;
    QCheckBox *checkBoxDebug;
    QCheckBox *checkBoxRecomputeTrajCost;
    QCheckBox *checkBoxWithShortCut;
    QCheckBox *checkBoxUseTRRT;
    MoveRobot *robotPosition;
    QWidget *tab;
    QGroupBox *groupBoxTimeModel;
    QPushButton *pushButtonCollision;
    QPushButton *pushButtonLocalPath;
    QPushButton *pushButtonCost;
    QLabel *labelCollision;
    QLabel *labelLocalPath;
    QLabel *labelTimeCost;
    QPushButton *pushButtonTestAll;
    QGroupBox *groupBoxInverseKinematiks;
    QPushButton *pushButtonAttMat;
    QFrame *line;

    void setupUi(QWidget *SideWindow)
    {
        if (SideWindow->objectName().isEmpty())
            SideWindow->setObjectName(QString::fromUtf8("SideWindow"));
        SideWindow->resize(491, 789);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(SideWindow->sizePolicy().hasHeightForWidth());
        SideWindow->setSizePolicy(sizePolicy);
        SideWindow->setMinimumSize(QSize(490, 0));
        horizontalLayout = new QHBoxLayout(SideWindow);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        Tabs = new QTabWidget(SideWindow);
        Tabs->setObjectName(QString::fromUtf8("Tabs"));
        sizePolicy.setHeightForWidth(Tabs->sizePolicy().hasHeightForWidth());
        Tabs->setSizePolicy(sizePolicy);
        Tabs->setMinimumSize(QSize(0, 0));
        Tabs->setMaximumSize(QSize(450, 16777215));
        MPlanning = new QWidget();
        MPlanning->setObjectName(QString::fromUtf8("MPlanning"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(1);
        sizePolicy1.setVerticalStretch(1);
        sizePolicy1.setHeightForWidth(MPlanning->sizePolicy().hasHeightForWidth());
        MPlanning->setSizePolicy(sizePolicy1);
        toolBox = new QToolBox(MPlanning);
        toolBox->setObjectName(QString::fromUtf8("toolBox"));
        toolBox->setGeometry(QRect(20, 15, 411, 491));
        PRMTab = new QWidget();
        PRMTab->setObjectName(QString::fromUtf8("PRMTab"));
        PRMTab->setGeometry(QRect(0, 0, 100, 30));
        groupBox_2 = new QGroupBox(PRMTab);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(20, 30, 171, 111));
        comboBox_2 = new QComboBox(groupBox_2);
        comboBox_2->setObjectName(QString::fromUtf8("comboBox_2"));
        comboBox_2->setGeometry(QRect(10, 30, 151, 26));
        checkBox_9 = new QCheckBox(groupBox_2);
        checkBox_9->setObjectName(QString::fromUtf8("checkBox_9"));
        checkBox_9->setGeometry(QRect(20, 70, 90, 23));
        groupBox_3 = new QGroupBox(PRMTab);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(210, 30, 171, 81));
        comboBox_3 = new QComboBox(groupBox_3);
        comboBox_3->setObjectName(QString::fromUtf8("comboBox_3"));
        comboBox_3->setGeometry(QRect(10, 30, 151, 26));
        groupBox_4 = new QGroupBox(PRMTab);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setGeometry(QRect(10, 150, 371, 111));
        radioButton = new QRadioButton(groupBox_4);
        radioButton->setObjectName(QString::fromUtf8("radioButton"));
        radioButton->setGeometry(QRect(20, 40, 103, 21));
        radioButton_2 = new QRadioButton(groupBox_4);
        radioButton_2->setObjectName(QString::fromUtf8("radioButton_2"));
        radioButton_2->setGeometry(QRect(290, 40, 103, 21));
        radioButton_3 = new QRadioButton(groupBox_4);
        radioButton_3->setObjectName(QString::fromUtf8("radioButton_3"));
        radioButton_3->setGeometry(QRect(160, 40, 103, 21));
        radioButton_4 = new QRadioButton(groupBox_4);
        radioButton_4->setObjectName(QString::fromUtf8("radioButton_4"));
        radioButton_4->setGeometry(QRect(20, 80, 103, 21));
        radioButton_5 = new QRadioButton(groupBox_4);
        radioButton_5->setObjectName(QString::fromUtf8("radioButton_5"));
        radioButton_5->setGeometry(QRect(160, 80, 103, 21));
        toolBox->addItem(PRMTab, QString::fromUtf8("PRM"));
        DiffTab = new QWidget();
        DiffTab->setObjectName(QString::fromUtf8("DiffTab"));
        DiffTab->setGeometry(QRect(0, 0, 411, 421));
        groupBox = new QGroupBox(DiffTab);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(10, 110, 391, 231));
        isAddingCycles = new QCheckBox(groupBox);
        isAddingCycles->setObjectName(QString::fromUtf8("isAddingCycles"));
        isAddingCycles->setGeometry(QRect(250, 80, 111, 23));
        expansionMethod = new QComboBox(groupBox);
        expansionMethod->setObjectName(QString::fromUtf8("expansionMethod"));
        expansionMethod->setGeometry(QRect(20, 40, 331, 26));
        isManualRefiRadius = new QCheckBox(groupBox);
        isManualRefiRadius->setObjectName(QString::fromUtf8("isManualRefiRadius"));
        isManualRefiRadius->setGeometry(QRect(20, 80, 161, 21));
        horizontalSliderExtentionStep = new QSlider(groupBox);
        horizontalSliderExtentionStep->setObjectName(QString::fromUtf8("horizontalSliderExtentionStep"));
        horizontalSliderExtentionStep->setGeometry(QRect(180, 130, 191, 23));
        horizontalSliderExtentionStep->setMaximum(10000000);
        horizontalSliderExtentionStep->setOrientation(Qt::Horizontal);
        doubleSpinBoxExtentionStep = new QDoubleSpinBox(groupBox);
        doubleSpinBoxExtentionStep->setObjectName(QString::fromUtf8("doubleSpinBoxExtentionStep"));
        doubleSpinBoxExtentionStep->setGeometry(QRect(90, 130, 71, 25));
        doubleSpinBoxExtentionStep->setDecimals(4);
        doubleSpinBoxExtentionStep->setMinimum(0);
        doubleSpinBoxExtentionStep->setMaximum(50);
        checkBox = new QCheckBox(groupBox);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));
        checkBox->setGeometry(QRect(10, 170, 71, 21));
        doubleSpinBoxBias = new QDoubleSpinBox(groupBox);
        doubleSpinBoxBias->setObjectName(QString::fromUtf8("doubleSpinBoxBias"));
        doubleSpinBoxBias->setGeometry(QRect(90, 170, 71, 25));
        doubleSpinBoxBias->setMaximum(1);
        doubleSpinBoxBias->setSingleStep(0.01);
        horizontalSliderBias = new QSlider(groupBox);
        horizontalSliderBias->setObjectName(QString::fromUtf8("horizontalSliderBias"));
        horizontalSliderBias->setGeometry(QRect(180, 170, 191, 22));
        horizontalSliderBias->setOrientation(Qt::Horizontal);
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 130, 81, 17));
        isDiscardingNodes = new QCheckBox(DiffTab);
        isDiscardingNodes->setObjectName(QString::fromUtf8("isDiscardingNodes"));
        isDiscardingNodes->setGeometry(QRect(290, 50, 131, 23));
        isBalanced = new QCheckBox(DiffTab);
        isBalanced->setObjectName(QString::fromUtf8("isBalanced"));
        isBalanced->setGeometry(QRect(290, 20, 111, 23));
        isCostTransition = new QCheckBox(DiffTab);
        isCostTransition->setObjectName(QString::fromUtf8("isCostTransition"));
        isCostTransition->setGeometry(QRect(150, 80, 131, 23));
        isExpandControl = new QCheckBox(DiffTab);
        isExpandControl->setObjectName(QString::fromUtf8("isExpandControl"));
        isExpandControl->setGeometry(QRect(0, 50, 131, 23));
        isWithGoal = new QCheckBox(DiffTab);
        isWithGoal->setObjectName(QString::fromUtf8("isWithGoal"));
        isWithGoal->setGeometry(QRect(0, 20, 90, 23));
        isEST = new QCheckBox(DiffTab);
        isEST->setObjectName(QString::fromUtf8("isEST"));
        isEST->setGeometry(QRect(290, 80, 131, 23));
        labelMaxNodes = new QLabel(DiffTab);
        labelMaxNodes->setObjectName(QString::fromUtf8("labelMaxNodes"));
        labelMaxNodes->setGeometry(QRect(20, 380, 71, 17));
        isCostSpace = new QCheckBox(DiffTab);
        isCostSpace->setObjectName(QString::fromUtf8("isCostSpace"));
        isCostSpace->setGeometry(QRect(0, 80, 131, 23));
        isManhattan = new QCheckBox(DiffTab);
        isManhattan->setObjectName(QString::fromUtf8("isManhattan"));
        isManhattan->setGeometry(QRect(150, 50, 131, 23));
        isBidir = new QCheckBox(DiffTab);
        isBidir->setObjectName(QString::fromUtf8("isBidir"));
        isBidir->setGeometry(QRect(150, 20, 131, 23));
        lineEditMaxNodes = new QLineEdit(DiffTab);
        lineEditMaxNodes->setObjectName(QString::fromUtf8("lineEditMaxNodes"));
        lineEditMaxNodes->setGeometry(QRect(110, 380, 113, 22));
        toolBox->addItem(DiffTab, QString::fromUtf8("Diffusion"));
        Tabs->addTab(MPlanning, QString());
        Optimisation = new QWidget();
        Optimisation->setObjectName(QString::fromUtf8("Optimisation"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(Optimisation->sizePolicy().hasHeightForWidth());
        Optimisation->setSizePolicy(sizePolicy2);
        verticalLayoutWidget_2 = new QWidget(Optimisation);
        verticalLayoutWidget_2->setObjectName(QString::fromUtf8("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(10, 20, 421, 391));
        optimizeLayout = new QVBoxLayout(verticalLayoutWidget_2);
        optimizeLayout->setObjectName(QString::fromUtf8("optimizeLayout"));
        optimizeLayout->setContentsMargins(0, 0, 0, 0);
        Tabs->addTab(Optimisation, QString());
        Cost = new QWidget();
        Cost->setObjectName(QString::fromUtf8("Cost"));
        sizePolicy2.setHeightForWidth(Cost->sizePolicy().hasHeightForWidth());
        Cost->setSizePolicy(sizePolicy2);
        groupBoxGeneralCostSpace = new QGroupBox(Cost);
        groupBoxGeneralCostSpace->setObjectName(QString::fromUtf8("groupBoxGeneralCostSpace"));
        groupBoxGeneralCostSpace->setGeometry(QRect(20, 30, 401, 111));
        pushButtonShowTrajCost = new QPushButton(groupBoxGeneralCostSpace);
        pushButtonShowTrajCost->setObjectName(QString::fromUtf8("pushButtonShowTrajCost"));
        pushButtonShowTrajCost->setGeometry(QRect(30, 30, 141, 32));
        checkBoxRescale = new QCheckBox(groupBoxGeneralCostSpace);
        checkBoxRescale->setObjectName(QString::fromUtf8("checkBoxRescale"));
        checkBoxRescale->setGeometry(QRect(220, 30, 131, 21));
        pushButtonGridInGraph = new QPushButton(groupBoxGeneralCostSpace);
        pushButtonGridInGraph->setObjectName(QString::fromUtf8("pushButtonGridInGraph"));
        pushButtonGridInGraph->setGeometry(QRect(30, 70, 141, 32));
        pushButtonAStar = new QPushButton(groupBoxGeneralCostSpace);
        pushButtonAStar->setObjectName(QString::fromUtf8("pushButtonAStar"));
        pushButtonAStar->setGeometry(QRect(250, 70, 121, 32));
        isCostSpaceCopy = new QCheckBox(Cost);
        isCostSpaceCopy->setObjectName(QString::fromUtf8("isCostSpaceCopy"));
        isCostSpaceCopy->setGeometry(QRect(20, 10, 131, 23));
        groupBox_5 = new QGroupBox(Cost);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setGeometry(QRect(20, 140, 401, 611));
        spinBoxJoint = new QSpinBox(groupBox_5);
        spinBoxJoint->setObjectName(QString::fromUtf8("spinBoxJoint"));
        spinBoxJoint->setGeometry(QRect(160, 80, 71, 25));
        label_8 = new QLabel(groupBox_5);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(10, 80, 131, 17));
        HRITaskSpace = new QGroupBox(groupBox_5);
        HRITaskSpace->setObjectName(QString::fromUtf8("HRITaskSpace"));
        HRITaskSpace->setGeometry(QRect(20, 440, 361, 151));
        whichTestBox = new QComboBox(HRITaskSpace);
        whichTestBox->setObjectName(QString::fromUtf8("whichTestBox"));
        whichTestBox->setGeometry(QRect(20, 90, 111, 26));
        enableHriTS = new QCheckBox(HRITaskSpace);
        enableHriTS->setObjectName(QString::fromUtf8("enableHriTS"));
        enableHriTS->setGeometry(QRect(10, 30, 171, 21));
        pushButtonWorkspacePath = new QPushButton(HRITaskSpace);
        pushButtonWorkspacePath->setObjectName(QString::fromUtf8("pushButtonWorkspacePath"));
        pushButtonWorkspacePath->setGeometry(QRect(20, 60, 121, 32));
        pushButtonHoleMotion = new QPushButton(HRITaskSpace);
        pushButtonHoleMotion->setObjectName(QString::fromUtf8("pushButtonHoleMotion"));
        pushButtonHoleMotion->setGeometry(QRect(170, 60, 121, 32));
        horizontalSliderVisibility = new QSlider(HRITaskSpace);
        horizontalSliderVisibility->setObjectName(QString::fromUtf8("horizontalSliderVisibility"));
        horizontalSliderVisibility->setGeometry(QRect(190, 130, 161, 22));
        horizontalSliderVisibility->setMaximum(500);
        horizontalSliderVisibility->setOrientation(Qt::Horizontal);
        doubleSpinBoxVisibility = new QDoubleSpinBox(HRITaskSpace);
        doubleSpinBoxVisibility->setObjectName(QString::fromUtf8("doubleSpinBoxVisibility"));
        doubleSpinBoxVisibility->setGeometry(QRect(100, 130, 71, 25));
        doubleSpinBoxVisibility->setMaximum(300);
        label_6 = new QLabel(HRITaskSpace);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(30, 130, 61, 17));
        pushButtonHRITS = new QPushButton(groupBox_5);
        pushButtonHRITS->setObjectName(QString::fromUtf8("pushButtonHRITS"));
        pushButtonHRITS->setGeometry(QRect(250, 40, 121, 32));
        enableHri = new QCheckBox(groupBox_5);
        enableHri->setObjectName(QString::fromUtf8("enableHri"));
        enableHri->setGeometry(QRect(20, 30, 211, 21));
        groupBox_6 = new QGroupBox(groupBox_5);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        groupBox_6->setGeometry(QRect(20, 110, 361, 111));
        doubleSpinBoxCellSize = new QDoubleSpinBox(groupBox_6);
        doubleSpinBoxCellSize->setObjectName(QString::fromUtf8("doubleSpinBoxCellSize"));
        doubleSpinBoxCellSize->setGeometry(QRect(80, 70, 71, 21));
        doubleSpinBoxCellSize->setMaximum(10);
        doubleSpinBoxCellSize->setSingleStep(0.1);
        label = new QLabel(groupBox_6);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 70, 61, 17));
        pushButtonMakeGrid = new QPushButton(groupBox_6);
        pushButtonMakeGrid->setObjectName(QString::fromUtf8("pushButtonMakeGrid"));
        pushButtonMakeGrid->setGeometry(QRect(10, 30, 161, 32));
        pushButtonDeleteGrid = new QPushButton(groupBox_6);
        pushButtonDeleteGrid->setObjectName(QString::fromUtf8("pushButtonDeleteGrid"));
        pushButtonDeleteGrid->setGeometry(QRect(170, 30, 121, 32));
        horizontalSliderCellSize = new QSlider(groupBox_6);
        horizontalSliderCellSize->setObjectName(QString::fromUtf8("horizontalSliderCellSize"));
        horizontalSliderCellSize->setGeometry(QRect(170, 70, 171, 22));
        horizontalSliderCellSize->setOrientation(Qt::Horizontal);
        HRICSPlanner = new QGroupBox(groupBox_5);
        HRICSPlanner->setObjectName(QString::fromUtf8("HRICSPlanner"));
        HRICSPlanner->setGeometry(QRect(20, 230, 361, 211));
        checkBoxDrawGrid = new QCheckBox(HRICSPlanner);
        checkBoxDrawGrid->setObjectName(QString::fromUtf8("checkBoxDrawGrid"));
        checkBoxDrawGrid->setGeometry(QRect(230, 170, 91, 21));
        pushButtonComputeCost = new QPushButton(HRICSPlanner);
        pushButtonComputeCost->setObjectName(QString::fromUtf8("pushButtonComputeCost"));
        pushButtonComputeCost->setGeometry(QRect(110, 30, 121, 32));
        pushButtonAStaIn3DGrid = new QPushButton(HRICSPlanner);
        pushButtonAStaIn3DGrid->setObjectName(QString::fromUtf8("pushButtonAStaIn3DGrid"));
        pushButtonAStaIn3DGrid->setGeometry(QRect(20, 140, 111, 32));
        pushButtonResetCost = new QPushButton(HRICSPlanner);
        pushButtonResetCost->setObjectName(QString::fromUtf8("pushButtonResetCost"));
        pushButtonResetCost->setGeometry(QRect(230, 30, 121, 32));
        label_3 = new QLabel(HRICSPlanner);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 70, 71, 17));
        doubleSpinBoxZoneSize = new QDoubleSpinBox(HRICSPlanner);
        doubleSpinBoxZoneSize->setObjectName(QString::fromUtf8("doubleSpinBoxZoneSize"));
        doubleSpinBoxZoneSize->setGeometry(QRect(90, 70, 71, 21));
        doubleSpinBoxZoneSize->setMaximum(2);
        doubleSpinBoxZoneSize->setSingleStep(0.1);
        horizontalSliderZoneSize = new QSlider(HRICSPlanner);
        horizontalSliderZoneSize->setObjectName(QString::fromUtf8("horizontalSliderZoneSize"));
        horizontalSliderZoneSize->setGeometry(QRect(170, 70, 101, 22));
        horizontalSliderZoneSize->setOrientation(Qt::Horizontal);
        label_2 = new QLabel(HRICSPlanner);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 110, 61, 17));
        doubleSpinBoxDistance = new QDoubleSpinBox(HRICSPlanner);
        doubleSpinBoxDistance->setObjectName(QString::fromUtf8("doubleSpinBoxDistance"));
        doubleSpinBoxDistance->setGeometry(QRect(90, 100, 71, 25));
        doubleSpinBoxDistance->setMaximum(300);
        horizontalSliderDistance = new QSlider(HRICSPlanner);
        horizontalSliderDistance->setObjectName(QString::fromUtf8("horizontalSliderDistance"));
        horizontalSliderDistance->setGeometry(QRect(170, 100, 161, 22));
        horizontalSliderDistance->setMaximum(500);
        horizontalSliderDistance->setOrientation(Qt::Horizontal);
        checkBoxDrawDistance = new QCheckBox(HRICSPlanner);
        checkBoxDrawDistance->setObjectName(QString::fromUtf8("checkBoxDrawDistance"));
        checkBoxDrawDistance->setGeometry(QRect(230, 140, 121, 21));
        checkBoxHRICS_MOPL = new QCheckBox(HRICSPlanner);
        checkBoxHRICS_MOPL->setObjectName(QString::fromUtf8("checkBoxHRICS_MOPL"));
        checkBoxHRICS_MOPL->setGeometry(QRect(10, 30, 101, 21));
        checkBoxBBDist = new QCheckBox(HRICSPlanner);
        checkBoxBBDist->setObjectName(QString::fromUtf8("checkBoxBBDist"));
        checkBoxBBDist->setGeometry(QRect(160, 140, 87, 21));
        pushButtonHRICSRRT = new QPushButton(HRICSPlanner);
        pushButtonHRICSRRT->setObjectName(QString::fromUtf8("pushButtonHRICSRRT"));
        pushButtonHRICSRRT->setGeometry(QRect(20, 170, 113, 32));
        checkBoxCostBefore = new QCheckBox(Cost);
        checkBoxCostBefore->setObjectName(QString::fromUtf8("checkBoxCostBefore"));
        checkBoxCostBefore->setGeometry(QRect(170, 10, 221, 21));
        Tabs->addTab(Cost, QString());
        Greedy = new QWidget();
        Greedy->setObjectName(QString::fromUtf8("Greedy"));
        verticalLayoutWidget = new QWidget(Greedy);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 0, 421, 521));
        greedyLayout = new QVBoxLayout(verticalLayoutWidget);
        greedyLayout->setObjectName(QString::fromUtf8("greedyLayout"));
        greedyLayout->setContentsMargins(0, 0, 0, 0);
        checkBoxDebug = new QCheckBox(verticalLayoutWidget);
        checkBoxDebug->setObjectName(QString::fromUtf8("checkBoxDebug"));

        greedyLayout->addWidget(checkBoxDebug);

        checkBoxRecomputeTrajCost = new QCheckBox(verticalLayoutWidget);
        checkBoxRecomputeTrajCost->setObjectName(QString::fromUtf8("checkBoxRecomputeTrajCost"));

        greedyLayout->addWidget(checkBoxRecomputeTrajCost);

        checkBoxWithShortCut = new QCheckBox(verticalLayoutWidget);
        checkBoxWithShortCut->setObjectName(QString::fromUtf8("checkBoxWithShortCut"));

        greedyLayout->addWidget(checkBoxWithShortCut);

        checkBoxUseTRRT = new QCheckBox(verticalLayoutWidget);
        checkBoxUseTRRT->setObjectName(QString::fromUtf8("checkBoxUseTRRT"));

        greedyLayout->addWidget(checkBoxUseTRRT);

        Tabs->addTab(Greedy, QString());
        robotPosition = new MoveRobot();
        robotPosition->setObjectName(QString::fromUtf8("robotPosition"));
        sizePolicy2.setHeightForWidth(robotPosition->sizePolicy().hasHeightForWidth());
        robotPosition->setSizePolicy(sizePolicy2);
        robotPosition->setMaximumSize(QSize(16777215, 800));
        Tabs->addTab(robotPosition, QString());
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        groupBoxTimeModel = new QGroupBox(tab);
        groupBoxTimeModel->setObjectName(QString::fromUtf8("groupBoxTimeModel"));
        groupBoxTimeModel->setGeometry(QRect(20, 40, 411, 171));
        pushButtonCollision = new QPushButton(groupBoxTimeModel);
        pushButtonCollision->setObjectName(QString::fromUtf8("pushButtonCollision"));
        pushButtonCollision->setGeometry(QRect(10, 40, 113, 32));
        pushButtonLocalPath = new QPushButton(groupBoxTimeModel);
        pushButtonLocalPath->setObjectName(QString::fromUtf8("pushButtonLocalPath"));
        pushButtonLocalPath->setGeometry(QRect(10, 80, 113, 32));
        pushButtonCost = new QPushButton(groupBoxTimeModel);
        pushButtonCost->setObjectName(QString::fromUtf8("pushButtonCost"));
        pushButtonCost->setGeometry(QRect(10, 120, 113, 32));
        labelCollision = new QLabel(groupBoxTimeModel);
        labelCollision->setObjectName(QString::fromUtf8("labelCollision"));
        labelCollision->setGeometry(QRect(160, 50, 241, 17));
        labelLocalPath = new QLabel(groupBoxTimeModel);
        labelLocalPath->setObjectName(QString::fromUtf8("labelLocalPath"));
        labelLocalPath->setGeometry(QRect(160, 90, 241, 17));
        labelTimeCost = new QLabel(groupBoxTimeModel);
        labelTimeCost->setObjectName(QString::fromUtf8("labelTimeCost"));
        labelTimeCost->setGeometry(QRect(160, 130, 241, 17));
        pushButtonTestAll = new QPushButton(tab);
        pushButtonTestAll->setObjectName(QString::fromUtf8("pushButtonTestAll"));
        pushButtonTestAll->setGeometry(QRect(30, 10, 113, 32));
        groupBoxInverseKinematiks = new QGroupBox(tab);
        groupBoxInverseKinematiks->setObjectName(QString::fromUtf8("groupBoxInverseKinematiks"));
        groupBoxInverseKinematiks->setGeometry(QRect(20, 220, 411, 191));
        pushButtonAttMat = new QPushButton(groupBoxInverseKinematiks);
        pushButtonAttMat->setObjectName(QString::fromUtf8("pushButtonAttMat"));
        pushButtonAttMat->setGeometry(QRect(10, 40, 131, 32));
        Tabs->addTab(tab, QString());

        horizontalLayout->addWidget(Tabs);

        line = new QFrame(SideWindow);
        line->setObjectName(QString::fromUtf8("line"));
        line->setCursor(QCursor(Qt::SplitHCursor));
        line->setMouseTracking(true);
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line);


        retranslateUi(SideWindow);

        Tabs->setCurrentIndex(2);
        toolBox->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(SideWindow);
    } // setupUi

    void retranslateUi(QWidget *SideWindow)
    {
        SideWindow->setWindowTitle(QApplication::translate("SideWindow", "Form", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("SideWindow", "Planning strategy", 0, QApplication::UnicodeUTF8));
        comboBox_2->clear();
        comboBox_2->insertItems(0, QStringList()
         << QApplication::translate("SideWindow", "PRM", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SideWindow", "Visibility PRM", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SideWindow", "ACR", 0, QApplication::UnicodeUTF8)
        );
        checkBox_9->setText(QApplication::translate("SideWindow", "Oriented", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("SideWindow", "Connecting strategy", 0, QApplication::UnicodeUTF8));
        comboBox_3->clear();
        comboBox_3->insertItems(0, QStringList()
         << QApplication::translate("SideWindow", "Nb Edges", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SideWindow", "Distance", 0, QApplication::UnicodeUTF8)
        );
        groupBox_4->setTitle(QApplication::translate("SideWindow", "Sampling strategy", 0, QApplication::UnicodeUTF8));
        radioButton->setText(QApplication::translate("SideWindow", "Random", 0, QApplication::UnicodeUTF8));
        radioButton_2->setText(QApplication::translate("SideWindow", "Gaussian", 0, QApplication::UnicodeUTF8));
        radioButton_3->setText(QApplication::translate("SideWindow", "Halton", 0, QApplication::UnicodeUTF8));
        radioButton_4->setText(QApplication::translate("SideWindow", "Bridge", 0, QApplication::UnicodeUTF8));
        radioButton_5->setText(QApplication::translate("SideWindow", "OBPRM", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(PRMTab), QApplication::translate("SideWindow", "PRM", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("SideWindow", "Tree Expansion Method", 0, QApplication::UnicodeUTF8));
        isAddingCycles->setText(QApplication::translate("SideWindow", "Add Cycles", 0, QApplication::UnicodeUTF8));
        expansionMethod->clear();
        expansionMethod->insertItems(0, QStringList()
         << QApplication::translate("SideWindow", "Extend", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SideWindow", "Extend n Step", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SideWindow", "Connect", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SideWindow", "Cost Connect", 0, QApplication::UnicodeUTF8)
        );
        isManualRefiRadius->setText(QApplication::translate("SideWindow", "Manula refi. radius", 0, QApplication::UnicodeUTF8));
        checkBox->setText(QApplication::translate("SideWindow", "Bias", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("SideWindow", "Step (Delta)", 0, QApplication::UnicodeUTF8));
        isDiscardingNodes->setText(QApplication::translate("SideWindow", "Discard Nodes", 0, QApplication::UnicodeUTF8));
        isBalanced->setText(QApplication::translate("SideWindow", "Balanced", 0, QApplication::UnicodeUTF8));
        isCostTransition->setText(QApplication::translate("SideWindow", "Cost Transition", 0, QApplication::UnicodeUTF8));
        isExpandControl->setText(QApplication::translate("SideWindow", "Expand Control", 0, QApplication::UnicodeUTF8));
        isWithGoal->setText(QApplication::translate("SideWindow", "With Goal", 0, QApplication::UnicodeUTF8));
        isEST->setText(QApplication::translate("SideWindow", "EST Planner", 0, QApplication::UnicodeUTF8));
        labelMaxNodes->setText(QApplication::translate("SideWindow", "Max Node", 0, QApplication::UnicodeUTF8));
        isCostSpace->setText(QApplication::translate("SideWindow", "Cost Space", 0, QApplication::UnicodeUTF8));
        isManhattan->setText(QApplication::translate("SideWindow", "Manhatan", 0, QApplication::UnicodeUTF8));
        isBidir->setText(QApplication::translate("SideWindow", "Bidirectional", 0, QApplication::UnicodeUTF8));
        toolBox->setItemText(toolBox->indexOf(DiffTab), QApplication::translate("SideWindow", "Diffusion", 0, QApplication::UnicodeUTF8));
        Tabs->setTabText(Tabs->indexOf(MPlanning), QApplication::translate("SideWindow", "Planning", 0, QApplication::UnicodeUTF8));
        Tabs->setTabText(Tabs->indexOf(Optimisation), QApplication::translate("SideWindow", "Optimization", 0, QApplication::UnicodeUTF8));
        groupBoxGeneralCostSpace->setTitle(QApplication::translate("SideWindow", "General Cost Space", 0, QApplication::UnicodeUTF8));
        pushButtonShowTrajCost->setText(QApplication::translate("SideWindow", "Show Traj Cost", 0, QApplication::UnicodeUTF8));
        checkBoxRescale->setText(QApplication::translate("SideWindow", "Will not rescale", 0, QApplication::UnicodeUTF8));
        pushButtonGridInGraph->setText(QApplication::translate("SideWindow", "Grid In Graph", 0, QApplication::UnicodeUTF8));
        pushButtonAStar->setText(QApplication::translate("SideWindow", "Test A*", 0, QApplication::UnicodeUTF8));
        isCostSpaceCopy->setText(QApplication::translate("SideWindow", "Cost Space", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("SideWindow", "Hri", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("SideWindow", "Joint To Assign Cost", 0, QApplication::UnicodeUTF8));
        HRITaskSpace->setTitle(QApplication::translate("SideWindow", "Hri Task Space", 0, QApplication::UnicodeUTF8));
        whichTestBox->clear();
        whichTestBox->insertItems(0, QStringList()
         << QApplication::translate("SideWindow", "Distance", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SideWindow", "Comfort", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SideWindow", "Visibility", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("SideWindow", "Combine", 0, QApplication::UnicodeUTF8)
        );
        enableHriTS->setText(QApplication::translate("SideWindow", "Task Space (Akin)", 0, QApplication::UnicodeUTF8));
        pushButtonWorkspacePath->setText(QApplication::translate("SideWindow", "Workspace A*", 0, QApplication::UnicodeUTF8));
        pushButtonHoleMotion->setText(QApplication::translate("SideWindow", "Entire Path", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("SideWindow", "Visibility", 0, QApplication::UnicodeUTF8));
        pushButtonHRITS->setText(QApplication::translate("SideWindow", "enable TS HRI", 0, QApplication::UnicodeUTF8));
        enableHri->setText(QApplication::translate("SideWindow", "Human Robot Interactions", 0, QApplication::UnicodeUTF8));
        groupBox_6->setTitle(QApplication::translate("SideWindow", "My Grid", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SideWindow", "Cell Size", 0, QApplication::UnicodeUTF8));
        pushButtonMakeGrid->setText(QApplication::translate("SideWindow", "New HRICS Planner", 0, QApplication::UnicodeUTF8));
        pushButtonDeleteGrid->setText(QApplication::translate("SideWindow", "Delete Planner", 0, QApplication::UnicodeUTF8));
        HRICSPlanner->setTitle(QApplication::translate("SideWindow", "HRICS Planner", 0, QApplication::UnicodeUTF8));
        checkBoxDrawGrid->setText(QApplication::translate("SideWindow", "Draw Grid", 0, QApplication::UnicodeUTF8));
        pushButtonComputeCost->setText(QApplication::translate("SideWindow", "Compute Cost", 0, QApplication::UnicodeUTF8));
        pushButtonAStaIn3DGrid->setText(QApplication::translate("SideWindow", "HRICS A* ", 0, QApplication::UnicodeUTF8));
        pushButtonResetCost->setText(QApplication::translate("SideWindow", "Reset Cost", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SideWindow", "Zone Size", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("SideWindow", "Distance", 0, QApplication::UnicodeUTF8));
        checkBoxDrawDistance->setText(QApplication::translate("SideWindow", "Draw Distance", 0, QApplication::UnicodeUTF8));
        checkBoxHRICS_MOPL->setText(QApplication::translate("SideWindow", "HRICS MOPL", 0, QApplication::UnicodeUTF8));
        checkBoxBBDist->setText(QApplication::translate("SideWindow", "BB Dist", 0, QApplication::UnicodeUTF8));
        pushButtonHRICSRRT->setText(QApplication::translate("SideWindow", "HRICS RRT", 0, QApplication::UnicodeUTF8));
        checkBoxCostBefore->setText(QApplication::translate("SideWindow", "Compute Cost Before Collision", 0, QApplication::UnicodeUTF8));
        Tabs->setTabText(Tabs->indexOf(Cost), QApplication::translate("SideWindow", "Cost", 0, QApplication::UnicodeUTF8));
        checkBoxDebug->setText(QApplication::translate("SideWindow", "Debug", 0, QApplication::UnicodeUTF8));
        checkBoxRecomputeTrajCost->setText(QApplication::translate("SideWindow", "Recompute Traj Cost", 0, QApplication::UnicodeUTF8));
        checkBoxWithShortCut->setText(QApplication::translate("SideWindow", "With Short Cut", 0, QApplication::UnicodeUTF8));
        checkBoxUseTRRT->setText(QApplication::translate("SideWindow", "With T-RRT", 0, QApplication::UnicodeUTF8));
        Tabs->setTabText(Tabs->indexOf(Greedy), QApplication::translate("SideWindow", "Utils", 0, QApplication::UnicodeUTF8));
        Tabs->setTabText(Tabs->indexOf(robotPosition), QApplication::translate("SideWindow", "Robot", 0, QApplication::UnicodeUTF8));
        groupBoxTimeModel->setTitle(QApplication::translate("SideWindow", "Time", 0, QApplication::UnicodeUTF8));
        pushButtonCollision->setText(QApplication::translate("SideWindow", "Collision", 0, QApplication::UnicodeUTF8));
        pushButtonLocalPath->setText(QApplication::translate("SideWindow", "LocalPath", 0, QApplication::UnicodeUTF8));
        pushButtonCost->setText(QApplication::translate("SideWindow", "Cost", 0, QApplication::UnicodeUTF8));
        labelCollision->setText(QApplication::translate("SideWindow", "Collision Time", 0, QApplication::UnicodeUTF8));
        labelLocalPath->setText(QApplication::translate("SideWindow", "Local Path Time", 0, QApplication::UnicodeUTF8));
        labelTimeCost->setText(QApplication::translate("SideWindow", "Cost Time", 0, QApplication::UnicodeUTF8));
        pushButtonTestAll->setText(QApplication::translate("SideWindow", "Test All", 0, QApplication::UnicodeUTF8));
        groupBoxInverseKinematiks->setTitle(QApplication::translate("SideWindow", "Inverse Kinematiks", 0, QApplication::UnicodeUTF8));
        pushButtonAttMat->setText(QApplication::translate("SideWindow", "Set Att Matrix", 0, QApplication::UnicodeUTF8));
        Tabs->setTabText(Tabs->indexOf(tab), QApplication::translate("SideWindow", "Model", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SideWindow: public Ui_SideWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // SIDEWINDOW_H
