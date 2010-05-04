/*
 *  qtMotionPlanner.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef QT_MOTIONPLANNER_H
#define QT_MOTIONPLANNER_H

#include "../qtLibrary.h"
#include "../../p3d/env.hpp"
#include "mainwindow.hpp"

namespace Ui
{
    class MotionPlanner;
}

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class MotionPlanner : public QWidget
{
    Q_OBJECT
	
public:
    MotionPlanner(QWidget *parent = 0);
    ~MotionPlanner();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	
private slots:
// Optim -----------------------------
    void computeGrid();
    void runMultiSmooth();
    void optimizeCost();
    void shortCutCost();
    void removeRedundant();
    void extractBestTraj();
	void setCostCriterium(int choise);
	
// Multi-Run -------------------------
    void saveContext();
    void printContext();
    void printAllContext();
    void resetContext();
    void setToSelected();
    void runAllRRT();
    void runAllGreedy();
    void showHistoWindow();
	
// General ---------------------------
	void checkAllEdges();
	
private:
    Ui::MotionPlanner *m_ui;
	
	MainWindow *m_mainWindow;
	
	QListWidget* contextList;
    std::vector<QListWidgetItem*> itemList;

#ifdef QWT
	HistoWindow* histoWin;
#endif
	
	void initDiffusion();
    void initPRM();
	void initMultiRun();
	void initOptim();
	void initGeneral();
};

/**
 * @ingroup qtWindow
 * @brief Smoothing thread class 
 */
class SmoothThread: public QThread
{
	Q_OBJECT
	
public:
	SmoothThread(QObject* parent = 0);
	
protected:
	void run();
	
};
	
#endif
