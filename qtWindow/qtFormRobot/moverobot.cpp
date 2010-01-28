#include "moverobot.hpp"
#include "ui_moverobot.hpp"
#include "../../planner_cxx/API/planningAPI.hpp"
#include "../cppToQt.hpp"

using namespace std;
using namespace tr1;

MoveRobot::MoveRobot(QWidget *parent) :
        QWidget(parent),
        m_ui(new Ui::MoveRobot)
{
    m_ui->setupUi(this);
    
    for(int i=0;i<XYZ_ENV->nr;i++)
    {
        if(i==0)
        {
            mTabWidget = new QTabWidget(this);
        }

        Robot* ptrRob = new Robot(XYZ_ENV->robot[i]);

        this->initSliders( newGridLayoutForRobot(ptrRob)  , ptrRob );
        shared_ptr<Configuration> ptrConf = ptrRob->getInitialPosition();
        //        ptrConf->print();
        setSliders(ptrConf);
        ptrRob->setAndUpdate(*ptrConf);

        std::string str = "g3d_draw_allwin_active";
        write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    }

}

MoveRobot::~MoveRobot()
{
    delete m_ui;
}

void MoveRobot::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

int MoveRobot::calc_real_dof(void)
{
    int nrd;
    int njnt,i,j,k;
    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

    nrd = robotPt->nb_user_dof;
    njnt = p3d_get_robot_njnt();

    if(njnt > MAX_NJNTS_IN_ROBOTFORM) {
        return 0;
    }

    for(i=0; i<=njnt; i++) {
        for(j=0; j<robotPt->joints[i]->dof_equiv_nbr; j++) {
            k = robotPt->joints[i]->index_dof + j;
            if((! p3d_jnt_get_dof_is_user(robotPt->joints[i], j)) &&
               (robotPt->cntrt_manager->in_cntrt[k] == 1)) {
                nrd++;
            }
        }
    }

    return nrd;
}

void MoveRobot::initSliders(QGridLayout *myGrid , Robot* ptrRob )
{
    //    int       i, j, k, ir, ord;
    int k;
    int njnt, nb_dof;
    configPt robot_pos_deg;
    p3d_rob *robotPt;
    p3d_jnt * jntPt;

    vector<DofSlider*> newSliders;
    mSliders.push_back( newSliders );

    nb_dof =    ptrRob->getRobotStruct()->nb_dof; //p3d_get_robot_ndof();
    njnt =      ptrRob->getRobotStruct()->njoints; //p3d_get_robot_njnt();
    //    ir =        ptrRob->getRobotStruct()->num; //p3d_get_desc_curnum(P3D_ROBOT);
    robotPt =   ptrRob->getRobotStruct(); //(p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

    if(calc_real_dof() > 0)
    {
        for(int i=0; i<=njnt; i++)
        {
            jntPt = robotPt->joints[i];

            for(int j=0; j<jntPt->dof_equiv_nbr; j++)
            {
                k = jntPt->index_dof + j;

                if((p3d_jnt_get_dof_is_user(jntPt,j)) || (robotPt->cntrt_manager->in_cntrt[k] == 1))
                {
                    mSliders.back().push_back( makeSlider(myGrid,ptrRob,jntPt,j) );

                    if (robotPt->cntrt_manager->in_cntrt[k] == 2)
                    {
                        mSliders.back().back()->doubleSpinBox->setDisabled(true);
                        mSliders.back().back()->horizontalSlider->setDisabled(true);
                    }
                }
            }
        }
    }

    QSpacerItem *verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
    myGrid->addItem(verticalSpacer, k, 0, 1, 1);
}

void MoveRobot::setSliders(shared_ptr<Configuration> ptrConfRad)
{
    string RobotName = ptrConfRad->getRobot()->getName();

    shared_ptr<Configuration> ptrConfDeg = ptrConfRad->getConfigInDegree();

    //    cout << "--------" << endl;
    //    cout << RobotName  << endl;
    //    cout << "--------" << endl;

    for(int numRob=0;numRob<mSliders.size();numRob++)
    {
//        cout << mSliders[numRob][0]->getRobot()->getName() << mSliders.size() <<endl;
        if(mSliders[numRob][0]->getRobot()->getName().compare( RobotName ) == 0 )
        {
            p3d_rob* robotPt = ptrConfDeg->getRobot()->getRobotStruct();
            int numDof = 0;

            if(calc_real_dof() > 0)
            {
                for(int i=0; i<=robotPt->njoints; i++)
                {
                    p3d_jnt* jntPt = robotPt->joints[i];

                    for(int j=0; j<jntPt->dof_equiv_nbr; j++)
                    {
                        int k = jntPt->index_dof + j;

                        if((p3d_jnt_get_dof_is_user(jntPt,j)) || (robotPt->cntrt_manager->in_cntrt[k] == 1))
                        {
//                            cout << ptrConfDeg->getConfigStruct() << endl;
                            mSliders[numRob][numDof]->connector->setValue( ptrConfDeg->at(k) );
                            numDof++;

//                            if(RobotName.compare("ROBOT") == 0)
//                            {
//                                cout << " Jidof dof[" << k <<"] = "<< ptrConfDeg->at(k) << endl;
//                            }

                            if (robotPt->cntrt_manager->in_cntrt[k] == 2)
                            {
                                //                                mSliders.back().back()->doubleSpinBox->setDisabled(true);
                                //                                mSliders.back().back()->horizontalSlider->setDisabled(true);
                            }
                        }
                    }
                }
            }
        }
    }
}

QGridLayout* MoveRobot::newGridLayoutForRobot(Robot* ptrRob)
{
    QString robotName(ptrRob->getName().c_str());
    QWidget *tab;
    QVBoxLayout *robotLayoutTab;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QHBoxLayout *horizontalLayoutScrollArea;
    QGridLayout *gridLayout;

    tab = new QWidget;

    robotLayoutTab = new QVBoxLayout(tab);
    scrollArea = new QScrollArea(tab);
    scrollArea->setWidgetResizable(true);
    scrollAreaWidgetContents = new QWidget();
    scrollAreaWidgetContents->setGeometry(QRect(0, 0, 700, 250));
    horizontalLayoutScrollArea = new QHBoxLayout(scrollAreaWidgetContents);

    gridLayout = new QGridLayout();
    horizontalLayoutScrollArea->addLayout(gridLayout);
    scrollArea->setWidget(scrollAreaWidgetContents);
    robotLayoutTab->addWidget(scrollArea);

    mTabWidget->addTab(tab, QString());
    mTabWidget->setTabText(mTabWidget->indexOf(tab), robotName );

    m_ui->gridLayout->addWidget(mTabWidget);

    robotLayoutTab->addWidget(new QPushButton);

    return gridLayout;
}

DofSlider* MoveRobot::makeSlider(QGridLayout *myGrid, Robot* ptrRob, p3d_jnt *jntPt,int DofNumOnJnt)
{
    int DofNum = jntPt->index_dof + DofNumOnJnt;
    const char* name = p3d_jnt_get_dof_name(jntPt, DofNumOnJnt);
    QString dofName(name);

    double max;
    double min;

    p3d_jnt_get_dof_bounds_deg(jntPt, DofNumOnJnt, &min, &max);

    DofSlider* slider = new DofSlider;

    slider->setDofNum(DofNum);
    slider->setRobot(ptrRob);

    // Label
    slider->label = new QLabel(this);
    slider->label->setObjectName("LabelDof");
    slider->label->setText(dofName);

    myGrid->addWidget(slider->label, DofNum, 0, 1, 1);

    // Slider
    slider->doubleSpinBox = new QDoubleSpinBox(this);
    slider->doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));
    slider->doubleSpinBox->setDecimals(3);
    slider->doubleSpinBox->setSingleStep(1e-03);
    slider->doubleSpinBox->setMaximum(max);
    slider->doubleSpinBox->setMinimum(min);

    // SpinBox
    myGrid->addWidget(slider->doubleSpinBox, DofNum, 1, 1, 1);

    slider->horizontalSlider = new QSlider(this);
    slider->horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
    slider->horizontalSlider->setOrientation(Qt::Horizontal);

    slider->horizontalSlider->setMaximum(10000);

    myGrid->addWidget(slider->horizontalSlider, DofNum, 2, 1, 1);

    // Connector
    slider->connector = new QtShiva::SpinBoxSliderConnector(
            this, slider->doubleSpinBox, slider->horizontalSlider );

    connect(slider->connector,SIGNAL(valueChanged(double)),slider,SLOT(dofValueChanged(double)));

    return slider;
}

void DofSlider::dofValueChanged(double value)
{
    //    std::string str = "ChangeDof";
    //    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    //    CB_position_obj(NULL,value);

    configPt p=NULL, p_deg=NULL;
    int i,nb_dof,ir,i_dof;
    p3d_rob *robotPt;
    p3d_jnt *jntPt;
    int ncol=0;
    int I_can;
    int nreshoot;   // <-modif Juan
    int *ikSol = NULL;

    double val = value; //fl_get_slider_value(ob);
    int arg = mDofNum;

    //    ir = p3d_get_desc_curnum(P3D_ROBOT);
    //    robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    //    nb_dof = p3d_get_robot_ndof();

    nb_dof =    mRobot->getRobotStruct()->nb_dof; //p3d_get_robot_ndof();
    ir =        mRobot->getRobotStruct()->num; //p3d_get_desc_curnum(P3D_ROBOT);
    robotPt =   mRobot->getRobotStruct(); //(p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

    p = p3d_alloc_config(robotPt);
    //    p = mRobot->getNewConfig()->getConfigStruct();
    //     p_deg = p3d_alloc_config(robotPt);


    //        if(fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ) == 1){
    //          p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_POS, &p_deg);
    //          ikSol = robotPt->ikSolPos;
    //        }
    //        else{
    //          p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_GOTO, &p_deg);
    //          ikSol = robotPt->ikSolGoto;
    //        }

    p_deg = p3d_get_robot_config_deg(robotPt);
    //    p3d_convert_config_rad_to_deg(robotPt, robotPt->ROBOT_POS, &p_deg)
    //    p_deg = mRobot->getCurrentPos()->getConfigInDegree()->getConfigStruct();

    p_deg[arg] = val;
    p3d_convert_config_deg_to_rad(robotPt, p_deg, &p);

    /*update the configuration of the current robot */
    /* sustitution pour la partie de la fonction dans l'anciene version de Move3D */
    /*   p3d_set_and_update_robot_conf(p); */ /* <- dans la fonction  remplacee */

    p3d_set_robot_config(robotPt, p);

    //   I_can = p3d_update_robot_pos();
    I_can = p3d_update_this_robot_pos_multisol(robotPt, NULL, 0, ikSol);

    if (robotPt->cntrt_manager->cntrts != NULL)
    {
        // modif Juan
        nreshoot = 0;
        if(!I_can && p3d_get_RLG())
        {
            if(robotPt->cntrt_manager->in_cntrt[arg] == 1)
            {
                while(!I_can && (nreshoot < 100))
                {
                    if(p3d_random_loop_generator_without_parallel(robotPt, p))
                    {
                        I_can = p3d_update_robot_pos();
                    }
                    nreshoot++;
                }
            }
        }
        //         fmodif Juan
        if(I_can)
        {
            p3d_get_robot_config_into(robotPt, &p);
            p3d_get_robot_config_deg_into(robotPt, &p_deg);
            for(i=0; i<nb_dof; i++)
            {
                //                if (ROBOTS_FORM[ir].POSITION_OBJ[i] != NULL)
                //                {
                //                    fl_set_slider_value(ROBOTS_FORM[ir].POSITION_OBJ[i], p_deg[i]);
                //                }
            }
            //            p3d_copy_config_into(robotPt, p_deg, &last_p_deg[ir]);
        }
        else
        {
            //            p3d_copy_config_into(robotPt, last_p_deg[ir], &p_deg);
            p3d_convert_config_deg_to_rad(robotPt, p_deg, &p);
            for(i=0; i<nb_dof; i++)
            {
                //                if (ROBOTS_FORM[ir].POSITION_OBJ[i] != NULL)
                //                {
                //                    fl_set_slider_value(ROBOTS_FORM[ir].POSITION_OBJ[i], p_deg[i]);
                //                }
            }
            jntPt = p3d_robot_dof_to_jnt(robotPt, arg, &i_dof);
            p3d_jnt_set_dof_deg(jntPt, i_dof, p_deg[arg]);  /* ceneccasy lines for some cases of cntrts !!! */
            p3d_update_this_robot_pos_without_cntrt(robotPt);
        }
    }

    if (ENV.getBool(Env::isCostSpace))
    {
        std::cout << "Cost = " << p3d_GetConfigCost(robotPt,p) << std::endl;
    }

    /* collision checking */
    if( g3d_get_KCD_CHOICE_IS_ACTIVE() )
    {
        if(G3D_ACTIVE_CC)
        {
            ncol = p3d_col_test_choice();
        }
    }
    else
    {
        if(G3D_ACTIVE_CC)
        {
            ncol = p3d_col_test_all();
        }
    }

    g3d_set_draw_coll(ncol);

    /* update the field current position or goal position of the
       current robot depending on field GOTO_OBJ */
    //    if(fl_get_choice(ROBOTS_FORM[ir].GOTO_OBJ) == 1){
    //        p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_POS));
    //    }
    //    else{
    //        p3d_copy_config_into(robotPt, p, &(robotPt->ROBOT_GOTO));
    //    }

    /* update trajectory if track_traj mode is activated */
    //    if (ROBOTS_FORM[ir].TRACK_TRAJ)
    //    {
    //        CB_stop_obj(ob,0);
    //        CB_stop_optim_obj(ob,0);
    //        if (!p3d_trackCurrentTraj(robotPt,3, 0.001, p3d_get_d0(),p3d_get_QUICK_DESCENT()))
    //        {
    //            CB_specific_search_obj(ob,0);
    //        }
    //        p3d_set_and_update_robot_conf(p);
    //    }

    p3d_destroy_config(robotPt, p);
    p3d_destroy_config(robotPt, p_deg);

#ifndef WITH_XFORMS
    g3d_draw_allwin_active();
#else
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#endif
}


#include "moc_moverobot.cpp"
