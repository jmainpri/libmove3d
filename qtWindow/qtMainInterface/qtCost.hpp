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

#ifdef HRI_COSTSPACE
#include "qtHrics.hpp"
#endif

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

	void initCost();
	void initCostFunctions();
	void initThreshold();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	void setMotionWidget(MotionPlanner* ptrMLPW) { m_motionWidget = ptrMLPW; }
	
	HricsWidget* getHriWidget();
	
private slots:
	
// General Cost --------------------------------
	void setCostFunction(int costFunctionId);
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
	void runThresholdPlanner();
	
private:
    Ui::CostWidget*		m_ui;
	
	MotionPlanner*		m_motionWidget;
	MainWindow*			m_mainWindow;
	
#ifdef QWT
    BasicPlotWindow *plot;
#endif
	
};

/**
 * @ingroup qtWindow
 * @brief Planner thread class 
 */
class TestPlannerthread: public QThread
{
	Q_OBJECT
	
public:
	TestPlannerthread(QObject* parent = 0);
	
protected:
	void run();
	
};

#endif