/*
 *  qtCost.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef QT_COST_H
#define QT_COST_H

#include "../qtLibrary.h"
#include "mainwindow.hpp"
#include "qtMotionPlanner.hpp"

#ifdef QWT
#include "../qtPlot/basicPlotWindow.hpp"
#endif

namespace Ui
{
    class CostWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class CostWidget : public QWidget
{
    Q_OBJECT
	
public:
    CostWidget(QWidget *parent = 0);
    ~CostWidget();
	
	void initHRI();
	void initCost();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	void setMotionWidget(MotionPlanner* ptrMLPW) { m_motionWidget = ptrMLPW; }
	
private slots:

// HRI ----------------------------------------
// Natural
	void newNaturalCostSpace();
	void deleteNaturalCostSpace();
	
// CSpace
    void newHRIConfigSpace();
    void deleteHRIConfigSpace();
    void makeGridHRIConfigSpace();
    void makePlanHRIConfigSpace();
    void AStarInPlanHRIConfigSpace();
    void writeToOBPlane();
    void hriPlanRRT();

// Workspace
    void make3DHriGrid();
    void delete3DHriGrid();
    void computeGridCost();
    void resetGridCost();
    void AStarIn3DGrid();
    void HRICSRRT();
    void zoneSizeChanged();
    void resetRandomPoints();
	
 // Taskspace
    void computeWorkspacePath();
    void computeHoleMotion();
    void KDistance(double value);
    void KVisibility(double value);
    void make2DGrid();
	
    void enableHriSpace();
	void setWhichTestSlot(int test);
	
// General Cost --------------------------------
	void stonesGraph();
	void extractBestPath();
	void newGraphAndReComputeCost();
    void showTrajCost();
    void showHRITrajCost();
    void showTemperature();
    void setPlotedVector(std::vector<double> v);
    void putGridInGraph();
    void computeAStar();
	//void computeGridAndExtract();
	void graphSearchTest();
	
private:
    Ui::CostWidget *m_ui;
	
	MotionPlanner *m_motionWidget;
	MainWindow *m_mainWindow;
	
#ifdef QWT
    BasicPlotWindow *plot;
#endif
	
    void initHumanLike();
};

#endif