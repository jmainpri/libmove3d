/********************************************************************************
** Form generated from reading UI file 'sidewindow.ui'
**
** Created: Wed Nov 18 18:02:35 2009
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
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "../qtFormRobot/moverobot.hpp"

QT_BEGIN_NAMESPACE

class Ui_SideWindow
{
public:
    QHBoxLayout *horizontalLayout;
    QTabWidget *Tabs;
    QWidget *PRM;
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
    QWidget *Diffusion;
    QCheckBox *isWithGoal;
    QCheckBox *isExpandControl;
    QCheckBox *isBidir;
    QCheckBox *isManhattan;
    QCheckBox *isBalanced;
    QCheckBox *isDiscardingNodes;
    QGroupBox *groupBox;
    QCheckBox *isAddingCycles;
    QComboBox *expansionMethod;
    QCheckBox *isManualRefiRadius;
    QSlider *horizontalSliderExtentionStep;
    QDoubleSpinBox *doubleSpinBoxExtentionStep;
    QCheckBox *isCostSpace;
    QCheckBox *isCostTransition;
    QCheckBox *isEST;
    QLineEdit *lineEditMaxNodes;
    QLabel *labelMaxNodes;
    QWidget *Optimisation;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *optimizeLayout;
    QWidget *Cost;
    QGroupBox *groupBox_5;
    QCheckBox *checkBoxCostBefore;
    QPushButton *pushButtonShowTrajCost;
    QCheckBox *checkBoxRescale;
    QSlider *horizontalSlider_3;
    QLabel *label_4;
    QLineEdit *lineEdit_5;
    QLabel *label_3;
    QLabel *label_2;
    QSlider *horizontalSlider_5;
    QLineEdit *lineEdit_3;
    QLineEdit *lineEdit_4;
    QSlider *horizontalSlider_4;
    QWidget *Greedy;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *greedyLayout;
    QCheckBox *checkBoxDebug;
    QCheckBox *checkBoxRecomputeTrajCost;
    QCheckBox *checkBoxWithShortCut;
    QCheckBox *checkBoxUseTRRT;
    QWidget *HriCostSpace;
    QGroupBox *HRITaskSpace;
    QComboBox *whichTestBox;
    QCheckBox *enableHriTS;
    QPushButton *pushButtonWorkspacePath;
    QPushButton *pushButtonHoleMotion;
    QSlider *horizontalSliderDistance;
    QSlider *horizontalSliderVisibility;
    QDoubleSpinBox *doubleSpinBoxDistance;
    QDoubleSpinBox *doubleSpinBoxVisibility;
    QLabel *label;
    QLabel *label_5;
    QCheckBox *enableHri;
    QPushButton *pushButtonHRITS;
    QLabel *label_8;
    QSpinBox *spinBoxJoint;
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
        SideWindow->resize(499, 568);
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
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(Tabs->sizePolicy().hasHeightForWidth());
        Tabs->setSizePolicy(sizePolicy1);
        Tabs->setMinimumSize(QSize(0, 0));
        Tabs->setMaximumSize(QSize(450, 16777215));
        PRM = new QWidget();
        PRM->setObjectName(QString::fromUtf8("PRM"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(1);
        sizePolicy2.setVerticalStretch(1);
        sizePolicy2.setHeightForWidth(PRM->sizePolicy().hasHeightForWidth());
        PRM->setSizePolicy(sizePolicy2);
        groupBox_2 = new QGroupBox(PRM);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 10, 191, 111));
        comboBox_2 = new QComboBox(groupBox_2);
        comboBox_2->setObjectName(QString::fromUtf8("comboBox_2"));
        comboBox_2->setGeometry(QRect(20, 30, 151, 26));
        checkBox_9 = new QCheckBox(groupBox_2);
        checkBox_9->setObjectName(QString::fromUtf8("checkBox_9"));
        checkBox_9->setGeometry(QRect(20, 70, 90, 23));
        groupBox_3 = new QGroupBox(PRM);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(220, 10, 191, 81));
        comboBox_3 = new QComboBox(groupBox_3);
        comboBox_3->setObjectName(QString::fromUtf8("comboBox_3"));
        comboBox_3->setGeometry(QRect(20, 30, 151, 26));
        groupBox_4 = new QGroupBox(PRM);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setGeometry(QRect(10, 130, 421, 131));
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
        Tabs->addTab(PRM, QString());
        Diffusion = new QWidget();
        Diffusion->setObjectName(QString::fromUtf8("Diffusion"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(Diffusion->sizePolicy().hasHeightForWidth());
        Diffusion->setSizePolicy(sizePolicy3);
        isWithGoal = new QCheckBox(Diffusion);
        isWithGoal->setObjectName(QString::fromUtf8("isWithGoal"));
        isWithGoal->setGeometry(QRect(10, 10, 90, 23));
        isExpandControl = new QCheckBox(Diffusion);
        isExpandControl->setObjectName(QString::fromUtf8("isExpandControl"));
        isExpandControl->setGeometry(QRect(10, 40, 131, 23));
        isBidir = new QCheckBox(Diffusion);
        isBidir->setObjectName(QString::fromUtf8("isBidir"));
        isBidir->setGeometry(QRect(160, 10, 131, 23));
        isManhattan = new QCheckBox(Diffusion);
        isManhattan->setObjectName(QString::fromUtf8("isManhattan"));
        isManhattan->setGeometry(QRect(160, 40, 131, 23));
        isBalanced = new QCheckBox(Diffusion);
        isBalanced->setObjectName(QString::fromUtf8("isBalanced"));
        isBalanced->setGeometry(QRect(300, 10, 111, 23));
        isDiscardingNodes = new QCheckBox(Diffusion);
        isDiscardingNodes->setObjectName(QString::fromUtf8("isDiscardingNodes"));
        isDiscardingNodes->setGeometry(QRect(300, 40, 131, 23));
        groupBox = new QGroupBox(Diffusion);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(30, 120, 391, 181));
        isAddingCycles = new QCheckBox(groupBox);
        isAddingCycles->setObjectName(QString::fromUtf8("isAddingCycles"));
        isAddingCycles->setGeometry(QRect(250, 80, 111, 23));
        expansionMethod = new QComboBox(groupBox);
        expansionMethod->setObjectName(QString::fromUtf8("expansionMethod"));
        expansionMethod->setGeometry(QRect(30, 40, 331, 26));
        isManualRefiRadius = new QCheckBox(groupBox);
        isManualRefiRadius->setObjectName(QString::fromUtf8("isManualRefiRadius"));
        isManualRefiRadius->setGeometry(QRect(40, 80, 161, 21));
        horizontalSliderExtentionStep = new QSlider(groupBox);
        horizontalSliderExtentionStep->setObjectName(QString::fromUtf8("horizontalSliderExtentionStep"));
        horizontalSliderExtentionStep->setGeometry(QRect(150, 130, 221, 23));
        horizontalSliderExtentionStep->setOrientation(Qt::Horizontal);
        doubleSpinBoxExtentionStep = new QDoubleSpinBox(groupBox);
        doubleSpinBoxExtentionStep->setObjectName(QString::fromUtf8("doubleSpinBoxExtentionStep"));
        doubleSpinBoxExtentionStep->setGeometry(QRect(20, 130, 111, 25));
        doubleSpinBoxExtentionStep->setDecimals(8);
        doubleSpinBoxExtentionStep->setMinimum(0);
        doubleSpinBoxExtentionStep->setMaximum(50);
        isCostSpace = new QCheckBox(Diffusion);
        isCostSpace->setObjectName(QString::fromUtf8("isCostSpace"));
        isCostSpace->setGeometry(QRect(10, 70, 131, 23));
        isCostTransition = new QCheckBox(Diffusion);
        isCostTransition->setObjectName(QString::fromUtf8("isCostTransition"));
        isCostTransition->setGeometry(QRect(160, 70, 131, 23));
        isEST = new QCheckBox(Diffusion);
        isEST->setObjectName(QString::fromUtf8("isEST"));
        isEST->setGeometry(QRect(300, 70, 131, 23));
        lineEditMaxNodes = new QLineEdit(Diffusion);
        lineEditMaxNodes->setObjectName(QString::fromUtf8("lineEditMaxNodes"));
        lineEditMaxNodes->setGeometry(QRect(110, 330, 113, 22));
        labelMaxNodes = new QLabel(Diffusion);
        labelMaxNodes->setObjectName(QString::fromUtf8("labelMaxNodes"));
        labelMaxNodes->setGeometry(QRect(30, 330, 71, 17));
        Tabs->addTab(Diffusion, QString());
        Optimisation = new QWidget();
        Optimisation->setObjectName(QString::fromUtf8("Optimisation"));
        sizePolicy3.setHeightForWidth(Optimisation->sizePolicy().hasHeightForWidth());
        Optimisation->setSizePolicy(sizePolicy3);
        verticalLayoutWidget_2 = new QWidget(Optimisation);
        verticalLayoutWidget_2->setObjectName(QString::fromUtf8("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(10, 20, 421, 391));
        optimizeLayout = new QVBoxLayout(verticalLayoutWidget_2);
        optimizeLayout->setObjectName(QString::fromUtf8("optimizeLayout"));
        optimizeLayout->setContentsMargins(0, 0, 0, 0);
        Tabs->addTab(Optimisation, QString());
        Cost = new QWidget();
        Cost->setObjectName(QString::fromUtf8("Cost"));
        sizePolicy3.setHeightForWidth(Cost->sizePolicy().hasHeightForWidth());
        Cost->setSizePolicy(sizePolicy3);
        groupBox_5 = new QGroupBox(Cost);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setGeometry(QRect(10, 10, 421, 121));
        checkBoxCostBefore = new QCheckBox(groupBox_5);
        checkBoxCostBefore->setObjectName(QString::fromUtf8("checkBoxCostBefore"));
        checkBoxCostBefore->setGeometry(QRect(10, 30, 171, 21));
        pushButtonShowTrajCost = new QPushButton(groupBox_5);
        pushButtonShowTrajCost->setObjectName(QString::fromUtf8("pushButtonShowTrajCost"));
        pushButtonShowTrajCost->setGeometry(QRect(10, 70, 141, 32));
        checkBoxRescale = new QCheckBox(groupBox_5);
        checkBoxRescale->setObjectName(QString::fromUtf8("checkBoxRescale"));
        checkBoxRescale->setGeometry(QRect(200, 70, 131, 21));
        horizontalSlider_3 = new QSlider(Cost);
        horizontalSlider_3->setObjectName(QString::fromUtf8("horizontalSlider_3"));
        horizontalSlider_3->setGeometry(QRect(200, 330, 211, 23));
        horizontalSlider_3->setOrientation(Qt::Horizontal);
        label_4 = new QLabel(Cost);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 330, 59, 18));
        lineEdit_5 = new QLineEdit(Cost);
        lineEdit_5->setObjectName(QString::fromUtf8("lineEdit_5"));
        lineEdit_5->setGeometry(QRect(110, 320, 71, 28));
        label_3 = new QLabel(Cost);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(20, 360, 81, 18));
        label_2 = new QLabel(Cost);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(20, 390, 91, 18));
        horizontalSlider_5 = new QSlider(Cost);
        horizontalSlider_5->setObjectName(QString::fromUtf8("horizontalSlider_5"));
        horizontalSlider_5->setGeometry(QRect(200, 360, 211, 23));
        horizontalSlider_5->setOrientation(Qt::Horizontal);
        lineEdit_3 = new QLineEdit(Cost);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));
        lineEdit_3->setGeometry(QRect(110, 350, 71, 28));
        lineEdit_4 = new QLineEdit(Cost);
        lineEdit_4->setObjectName(QString::fromUtf8("lineEdit_4"));
        lineEdit_4->setGeometry(QRect(110, 380, 71, 28));
        horizontalSlider_4 = new QSlider(Cost);
        horizontalSlider_4->setObjectName(QString::fromUtf8("horizontalSlider_4"));
        horizontalSlider_4->setGeometry(QRect(200, 390, 211, 23));
        horizontalSlider_4->setOrientation(Qt::Horizontal);
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
        HriCostSpace = new QWidget();
        HriCostSpace->setObjectName(QString::fromUtf8("HriCostSpace"));
        HRITaskSpace = new QGroupBox(HriCostSpace);
        HRITaskSpace->setObjectName(QString::fromUtf8("HRITaskSpace"));
        HRITaskSpace->setGeometry(QRect(10, 90, 421, 201));
        whichTestBox = new QComboBox(HRITaskSpace);
        whichTestBox->setObjectName(QString::fromUtf8("whichTestBox"));
        whichTestBox->setGeometry(QRect(20, 80, 111, 26));
        enableHriTS = new QCheckBox(HRITaskSpace);
        enableHriTS->setObjectName(QString::fromUtf8("enableHriTS"));
        enableHriTS->setGeometry(QRect(20, 40, 171, 21));
        pushButtonWorkspacePath = new QPushButton(HRITaskSpace);
        pushButtonWorkspacePath->setObjectName(QString::fromUtf8("pushButtonWorkspacePath"));
        pushButtonWorkspacePath->setGeometry(QRect(272, 40, 131, 32));
        pushButtonHoleMotion = new QPushButton(HRITaskSpace);
        pushButtonHoleMotion->setObjectName(QString::fromUtf8("pushButtonHoleMotion"));
        pushButtonHoleMotion->setGeometry(QRect(272, 80, 131, 32));
        horizontalSliderDistance = new QSlider(HRITaskSpace);
        horizontalSliderDistance->setObjectName(QString::fromUtf8("horizontalSliderDistance"));
        horizontalSliderDistance->setGeometry(QRect(189, 130, 211, 22));
        horizontalSliderDistance->setOrientation(Qt::Horizontal);
        horizontalSliderVisibility = new QSlider(HRITaskSpace);
        horizontalSliderVisibility->setObjectName(QString::fromUtf8("horizontalSliderVisibility"));
        horizontalSliderVisibility->setGeometry(QRect(189, 160, 211, 22));
        horizontalSliderVisibility->setOrientation(Qt::Horizontal);
        doubleSpinBoxDistance = new QDoubleSpinBox(HRITaskSpace);
        doubleSpinBoxDistance->setObjectName(QString::fromUtf8("doubleSpinBoxDistance"));
        doubleSpinBoxDistance->setGeometry(QRect(100, 130, 71, 25));
        doubleSpinBoxDistance->setMaximum(300);
        doubleSpinBoxVisibility = new QDoubleSpinBox(HRITaskSpace);
        doubleSpinBoxVisibility->setObjectName(QString::fromUtf8("doubleSpinBoxVisibility"));
        doubleSpinBoxVisibility->setGeometry(QRect(100, 160, 71, 25));
        doubleSpinBoxVisibility->setMaximum(300);
        label = new QLabel(HRITaskSpace);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(30, 130, 61, 17));
        label_5 = new QLabel(HRITaskSpace);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(30, 160, 61, 17));
        enableHri = new QCheckBox(HriCostSpace);
        enableHri->setObjectName(QString::fromUtf8("enableHri"));
        enableHri->setGeometry(QRect(20, 10, 211, 21));
        pushButtonHRITS = new QPushButton(HriCostSpace);
        pushButtonHRITS->setObjectName(QString::fromUtf8("pushButtonHRITS"));
        pushButtonHRITS->setGeometry(QRect(250, 50, 121, 32));
        label_8 = new QLabel(HriCostSpace);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(90, 60, 141, 17));
        spinBoxJoint = new QSpinBox(HriCostSpace);
        spinBoxJoint->setObjectName(QString::fromUtf8("spinBoxJoint"));
        spinBoxJoint->setGeometry(QRect(20, 50, 56, 25));
        Tabs->addTab(HriCostSpace, QString());
        robotPosition = new MoveRobot();
        robotPosition->setObjectName(QString::fromUtf8("robotPosition"));
        sizePolicy3.setHeightForWidth(robotPosition->sizePolicy().hasHeightForWidth());
        robotPosition->setSizePolicy(sizePolicy3);
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

        Tabs->setCurrentIndex(7);


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
        Tabs->setTabText(Tabs->indexOf(PRM), QApplication::translate("SideWindow", "PRM", 0, QApplication::UnicodeUTF8));
        isWithGoal->setText(QApplication::translate("SideWindow", "With Goal", 0, QApplication::UnicodeUTF8));
        isExpandControl->setText(QApplication::translate("SideWindow", "Expand Control", 0, QApplication::UnicodeUTF8));
        isBidir->setText(QApplication::translate("SideWindow", "Bidirectional", 0, QApplication::UnicodeUTF8));
        isManhattan->setText(QApplication::translate("SideWindow", "Manhatan", 0, QApplication::UnicodeUTF8));
        isBalanced->setText(QApplication::translate("SideWindow", "Balanced", 0, QApplication::UnicodeUTF8));
        isDiscardingNodes->setText(QApplication::translate("SideWindow", "Discard Nodes", 0, QApplication::UnicodeUTF8));
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
        isCostSpace->setText(QApplication::translate("SideWindow", "Cost Space", 0, QApplication::UnicodeUTF8));
        isCostTransition->setText(QApplication::translate("SideWindow", "Cost Transition", 0, QApplication::UnicodeUTF8));
        isEST->setText(QApplication::translate("SideWindow", "EST Planner", 0, QApplication::UnicodeUTF8));
        labelMaxNodes->setText(QApplication::translate("SideWindow", "Max Node", 0, QApplication::UnicodeUTF8));
        Tabs->setTabText(Tabs->indexOf(Diffusion), QApplication::translate("SideWindow", "Diffusion", 0, QApplication::UnicodeUTF8));
        Tabs->setTabText(Tabs->indexOf(Optimisation), QApplication::translate("SideWindow", "Optimization", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("SideWindow", "General Cost Space", 0, QApplication::UnicodeUTF8));
        checkBoxCostBefore->setText(QApplication::translate("SideWindow", "Cost Before Collision", 0, QApplication::UnicodeUTF8));
        pushButtonShowTrajCost->setText(QApplication::translate("SideWindow", "Show Traj Cost", 0, QApplication::UnicodeUTF8));
        checkBoxRescale->setText(QApplication::translate("SideWindow", "Will not rescale", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("SideWindow", "Alpha", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SideWindow", "Temp. rate", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("SideWindow", "Initial Temp.", 0, QApplication::UnicodeUTF8));
        Tabs->setTabText(Tabs->indexOf(Cost), QApplication::translate("SideWindow", "Cost", 0, QApplication::UnicodeUTF8));
        checkBoxDebug->setText(QApplication::translate("SideWindow", "Debug", 0, QApplication::UnicodeUTF8));
        checkBoxRecomputeTrajCost->setText(QApplication::translate("SideWindow", "Recompute Traj Cost", 0, QApplication::UnicodeUTF8));
        checkBoxWithShortCut->setText(QApplication::translate("SideWindow", "With Short Cut", 0, QApplication::UnicodeUTF8));
        checkBoxUseTRRT->setText(QApplication::translate("SideWindow", "With T-RRT", 0, QApplication::UnicodeUTF8));
        Tabs->setTabText(Tabs->indexOf(Greedy), QApplication::translate("SideWindow", "Greedy", 0, QApplication::UnicodeUTF8));
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
        label->setText(QApplication::translate("SideWindow", "Distance", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("SideWindow", "Visibility", 0, QApplication::UnicodeUTF8));
        enableHri->setText(QApplication::translate("SideWindow", "Human Robot Interactions", 0, QApplication::UnicodeUTF8));
        pushButtonHRITS->setText(QApplication::translate("SideWindow", "enable TS HRI", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("SideWindow", "Joint To Assign Cost", 0, QApplication::UnicodeUTF8));
        Tabs->setTabText(Tabs->indexOf(HriCostSpace), QApplication::translate("SideWindow", "Hri", 0, QApplication::UnicodeUTF8));
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
