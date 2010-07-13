/*
 *  qtRobot.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef QT_ROBOT_H
#define QT_ROBOT_H

#include "../qtLibrary.h"
#include "qtFormRobot/moverobot.hpp"
#include "mainwindow.hpp"

namespace Ui
{
    class RobotWidget;
}

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class RobotWidget : public QWidget
{
    Q_OBJECT
	
public:
    RobotWidget(QWidget *parent = 0);
    ~RobotWidget();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	
	void initRobot();
	
	MoveRobot* getFormRobot();
	
private slots:
	
// Test Model -------------------------
	void costTest();
    void collisionsTest();
    void localpathsTest();
    void allTests();
    void setAttMatrix();

// Hri Planner ------------------------
#if defined (HRI_GENERALIZED_IK)
	void computeHriGikLARM() { this->computeHriGik(true); }
	void computeHriGikRARM() { this->computeHriGik(false); }
#endif
	
// Grab Object ------------------------
	
    void GrabObject();
    void ReleaseObject();
    void currentObjectChange(int i);
    void SetObjectToCarry();
	
// MISC -------------------------------
	void printCurrentPos();
	
#ifdef LIGHT_PLANNER
	void switchFKIK();
#endif
	
	void initVoxelCollisionChecker();
	void createVoxelCC();
	void deleteVoxelCC();
	
	void voxelCCTest();
	
private:
    Ui::RobotWidget *m_ui;
	
	MainWindow *m_mainWindow;
	
	std::vector<QString> mFreeFlyers;
	
	void initModel();
	
#if defined (HRI_GENERALIZED_IK)
	void computeHriGik(bool leftArm);
#endif
	
	
};

#endif
