/*
 *  qtRobot.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "qtRobot.hpp"
#include "ui_qtRobot.h"

#include "Util-pkg.h"

#ifdef CXX_PLANNER
#include "../../util/CppApi/MultiRun.hpp"
#include "../../util/CppApi/SaveContext.hpp"
#include "../../util/CppApi/testModel.hpp"
#endif

using namespace std;
using namespace tr1;

RobotWidget::RobotWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::RobotWidget)
{
    m_ui->setupUi(this);
	
	initModel();
	
}

RobotWidget::~RobotWidget()
{
    delete m_ui;
}

//---------------------------------------------------------------------
// Robot
//---------------------------------------------------------------------
void RobotWidget::initRobot()
{
	m_ui->formRobot->initAllForms(m_mainWindow->getOpenGL());	
}

//---------------------------------------------------------------------
// TEST MODEL
//---------------------------------------------------------------------
void RobotWidget::initModel()
{
    connect(m_ui->pushButtonCollision,SIGNAL(clicked()),this,SLOT(collisionsTest()));
    connect(m_ui->pushButtonLocalPath,SIGNAL(clicked()),this,SLOT(localpathsTest()));
    connect(m_ui->pushButtonCost,SIGNAL(clicked()),this,SLOT(costTest()));
    connect(m_ui->pushButtonTestAll,SIGNAL(clicked()),this,SLOT(allTests()));
    connect(m_ui->pushButtonSetObjectToCarry,SIGNAL(clicked()),this,SLOT(SetObjectToCarry()));
	
    connect(ENV.getObject(Env::numberOfCollisionPerSec),SIGNAL(valueChanged(QString)),m_ui->labelCollision,SLOT(setText(QString)));
    connect(ENV.getObject(Env::numberOfLocalPathPerSec),SIGNAL(valueChanged(QString)),m_ui->labelLocalPath,SLOT(setText(QString)));
    connect(ENV.getObject(Env::numberOfCostPerSec),SIGNAL(valueChanged(QString)),m_ui->labelTimeCost,SLOT(setText(QString)));
	
    connect(m_ui->pushButtonAttMat,SIGNAL(clicked()),this,SLOT(setAttMatrix()));
	
    QString RobotObjectToCarry("No Object");
	
    ENV.setString(Env::ObjectToCarry,RobotObjectToCarry);
	
    // Grab Object
    for(int i =0;i<XYZ_ENV->nr;i++)
    {
        if(XYZ_ENV->robot[i]->joints[1]->type == P3D_FREEFLYER )
        {
            if( XYZ_ENV->robot[i]->njoints == 1 )
            {
                QString FFname(XYZ_ENV->robot[i]->name);
                m_ui->comboBoxGrabObject->addItem(FFname);
                mFreeFlyers.push_back(FFname);
                //                cout<< " FreeFlyer = "  << XYZ_ENV->robot[i]->name << endl;
            }
        }
    }
	
    m_ui->comboBoxGrabObject->setCurrentIndex(0);
    connect(m_ui->comboBoxGrabObject, SIGNAL(currentIndexChanged(int)),this, SLOT(currentObjectChange(int))/*, Qt::DirectConnection*/);
	
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxIsWeightedRot,       Env::isWeightedRotation);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxFKSampling,          Env::FKShoot);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxFKDistance,          Env::FKDistance);
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxWeightedRot, m_ui->horizontalSliderWeightedRot , Env::RotationWeight );
	
	connect(m_ui->pushButtonGrabObject,SIGNAL(clicked()),this,SLOT(GrabObject()));
    connect(m_ui->pushButtonReleaseObject,SIGNAL(clicked()),this,SLOT(ReleaseObject()));
	
}

void RobotWidget::costTest()
{
    if(ENV.getBool(Env::isCostSpace))
    {
#ifdef CXX_PLANNER
        TestModel tests;
        tests.nbOfCostPerSeconds();
#endif
    }
}

void RobotWidget::collisionsTest()
{
#ifdef CXX_PLANNER
    TestModel tests;
    tests.nbOfColisionsPerSeconds();
#endif
}

void RobotWidget::localpathsTest()
{
#ifdef CXX_PLANNER	
    TestModel tests;
    tests.nbOfLocalPathsPerSeconds();
#endif
}

void RobotWidget::allTests()
{
#ifdef CXX_PLANNER		
    TestModel tests;
    tests.runAllTests();
#endif
}

void RobotWidget::setAttMatrix()
{
#ifdef LIGHT_PLANNER
    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    //  p3d_compute_attached_matrix_from_virt_obj(robotPt->ccCntrts[0]);
    for(int i = 0; i < robotPt->nbCcCntrts; i++)
	{
		p3d_compute_Tatt(robotPt->ccCntrts[i]);
		
		//      cout << "Tatt = " << endl;
		//      for (i = 0; i < 4; i++)
		//      {
		////        PrintInfo(("%+10.6f  %+10.6f  %+10.6f  %+10.6f\n",
		//               cout << robotPt->ccCntrts[i]->Tatt[i][0]
		//                       << robotPt->ccCntrts[i]->Tatt[i][1]
		//                       << robotPt->ccCntrts[i]->Tatt[i][2]
		//                       << robotPt->ccCntrts[i]->Tatt[i][3] << endl;
		//      }
		//      cout << endl;
    }
#endif
}

void RobotWidget::currentObjectChange(int i)
{
    if((mFreeFlyers.size() > 0) && (i != 0))
    {
		
        //        cout << "Env::ObjectToCarry  is "<< mFreeFlyers[i-1] << endl;
        ENV.setString(Env::ObjectToCarry,mFreeFlyers[i-1]);
    }
}

void RobotWidget::SetObjectToCarry()
{
#ifdef LIGHT_PLANNER
    if(mFreeFlyers.size() > 0)
    {
        p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
        p3d_set_object_to_carry(robotPt,ENV.getString(Env::ObjectToCarry).toStdString().c_str());
		
        // Set the dist of the object to the radius of the carried object
        robotPt->curObjectJnt->dist = robotPt->carriedObject->joints[1]->dist;
		
        double radius = 1.5;
        //take only x and y composantes of the base
        double dof[2][2];
        for(int i = 0; i < 2; i++){
            dof[i][0] = p3d_jnt_get_dof(robotPt->joints[1], i) - radius;
            dof[i][1] = p3d_jnt_get_dof(robotPt->joints[1], i) + radius;
        }
        for(int i = 0; i < 2; i++){
            p3d_jnt_set_dof_rand_bounds(robotPt->curObjectJnt, i, dof[i][0], dof[i][1]);
        }
		
    }
    else
    {
        p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
        // Set the dist of the object to the radius of the carried object
		
        cout << "Setting Dist of Joint : " << robotPt->joints[6]->name << endl;
        cout << "To Joint : "  << robotPt->joints[7]->name << endl;
		
        cout << robotPt->joints[7]->dist << "  Takes "  << robotPt->joints[6]->dist << endl;
		
        robotPt->joints[7]->dist = robotPt->joints[6]->dist;
    }
#endif
}

void RobotWidget::GrabObject()
{
#ifdef LIGHT_PLANNER
    if(mFreeFlyers.size() > 0)
    {
        p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
		//        p3d_rob *carriedObject;
		
        p3d_set_object_to_carry(robotPt,ENV.getString(Env::ObjectToCarry).toStdString().c_str());
		//        p3d_matrix4 saved;
		//        p3d_mat4Copy(robotPt->curObjectJnt->abs_pos,saved);
        p3d_mat4Copy(robotPt->carriedObject->joints[1]->abs_pos,robotPt->curObjectJnt->abs_pos);
        p3d_grab_object(robotPt,0);
		//        p3d_mat4Copy(saved,robotPt->curObjectJnt->abs_pos);
		//        configPt q = p3d_get_robot_config(robotPt);
		
		//        robotPt->ROBOT_POS = q;
		//        p3d_set_and_update_robot_conf(q);
        p3d_mat4Print(robotPt->ccCntrts[0]->Tatt,"curObject Grab");
    }
#endif
}

void RobotWidget::ReleaseObject()
{
#ifdef LIGHT_PLANNER
    //    m_ui->comboBoxGrabObject-
    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    p3d_release_object(robotPt);
    m_ui->comboBoxGrabObject->setCurrentIndex(0);
#endif
};