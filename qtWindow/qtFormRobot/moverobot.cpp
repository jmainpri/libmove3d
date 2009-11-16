#include "moverobot.hpp"
#include "ui_moverobot.hpp"
#include "../../planning_api/PlanningAPI.hpp"

using namespace std;

MoveRobot::MoveRobot(QWidget *parent) :
        QWidget(parent),
        m_ui(new Ui::MoveRobot)
{
    m_ui->setupUi(this);
//    QString dofName("truc");
//    Sliders.push_back(makeSlider(dofName));
    this->initSliders();
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

JointSlider* MoveRobot::makeSlider(QString& dofName)
{
//    cout << "Make slider " << dofName.toStdString() << endl;
    JointSlider* slider = new JointSlider;

    QHBoxLayout* SliderLayout = new QHBoxLayout;

    slider->label = new QLabel;
    slider->label->setText(dofName);
//    slider->label->textFormat();
//    slider.label->setObjectName(QString::fromUtf8("label"));
    QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(slider->label->sizePolicy().hasHeightForWidth());
    slider->label->setSizePolicy(sizePolicy);
    slider->label->setMinimumSize(QSize(200, 0));
//    slider->label->setMaximumSize(QSize(100, 0));

    SliderLayout->addWidget(slider->label);

    slider->lineEdit = new QLineEdit;
//    slider->lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
    QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy2.setHorizontalStretch(0);
    sizePolicy2.setVerticalStretch(0);
    sizePolicy2.setHeightForWidth(slider->lineEdit->sizePolicy().hasHeightForWidth());
    slider->lineEdit->setSizePolicy(sizePolicy2);
    slider->lineEdit->setMaximumSize(QSize(60, 16777215));
    slider->lineEdit->setMinimumSize(QSize(50, 0));

    SliderLayout->addWidget(slider->lineEdit);

    slider->horizontalScrollBar = new QScrollBar;
//    slider->horizontalScrollBar->setObjectName(QString::fromUtf8("horizontalScrollBar"));
    slider->horizontalScrollBar->setOrientation(Qt::Horizontal);

    SliderLayout->addWidget(slider->horizontalScrollBar);

    m_ui->verticalLayout->addLayout(SliderLayout);

    return slider;
}

void MoveRobot::initSliders()
{
    double     f_min=0.0, f_max=0.0;
    char      str[30];
    int       i, j, k, ir, ord;
    int njnt, nb_dof;
    configPt robot_pos_deg;
    p3d_rob *robotPt;
    p3d_jnt * jntPt;


    nb_dof = p3d_get_robot_ndof();
    njnt = p3d_get_robot_njnt();
    ir = p3d_get_desc_curnum(P3D_ROBOT);
    robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    /* convert current robot position into degree to initialize
     sliders */
//    last_p_deg[ir] = p3d_copy_config_rad_to_deg(robotPt, robotPt->ROBOT_POS);
//    robot_pos_deg = last_p_deg[ir];

    if(calc_real_dof() > 0)
    {
        ord = 0;
        for(i=0; i<=njnt; i++)
        {
            jntPt = robotPt->joints[i];

            for(j=0; j<jntPt->dof_equiv_nbr; j++)
            {
                k = jntPt->index_dof + j;

                if((p3d_jnt_get_dof_is_user(jntPt,j)) || (robotPt->cntrt_manager->in_cntrt[k] == 1))
                {
                    p3d_jnt_get_dof_bounds_deg(jntPt, j, &f_min, &f_max);

                    QString dofName(p3d_jnt_get_dof_name(jntPt, j));
                    Sliders.push_back(makeSlider(dofName));

                    Sliders.back()->max = f_max;
                    Sliders.back()->min = f_min;

                    if (robotPt->cntrt_manager->in_cntrt[k] == 2)
                    {
                        Sliders.back()->lineEdit->setDisabled(true);
                        Sliders.back()->horizontalScrollBar->setDisabled(true);
//                        fl_set_slider_size(ROBOTS_FORM[ir].POSITION_OBJ[k],1.0);
                    }
                    ord = ord+1;
                }
                else
                {
//                    ROBOTS_FORM[ir].POSITION_OBJ[k] = NULL;
                }
            }
        }
    }
}


#include "moc_moverobot.cpp"
