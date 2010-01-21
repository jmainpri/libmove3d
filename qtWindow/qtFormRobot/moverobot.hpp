#ifndef MOVEROBOT_HPP
#define MOVEROBOT_HPP

#include "../qtLibrary.h"
#include "../qtBase/SpinBoxSliderConnector_p.hpp"
#include "../../planner_cxx/API/planningAPI.hpp"

namespace Ui {
    class MoveRobot;
}

/**
  * One Dof slider
  */
class DofSlider : public QObject
{
    Q_OBJECT

public:
    DofSlider() {}
    ~DofSlider() {}

    QLabel *label;
    QDoubleSpinBox *doubleSpinBox;
    QSlider *horizontalSlider;

    QtShiva::SpinBoxSliderConnector *connector;

    void setValue(double value) { connector->setValue(value); }

    void setDofNum(int dofNum) { mDofNum = dofNum; }
    void setRobot(Robot* R) { mRobot = R; }

    Robot* getRobot() { return mRobot; }

public slots:
    void dofValueChanged(double value);

private:
    int     mDofNum;
    Robot*  mRobot;
};

/**
  * @ingroup qtMainWindow
  * @brief Makes the sliders in the qtRobot Module
  */
class MoveRobot : public QWidget {
    Q_OBJECT
public:
    MoveRobot(QWidget *parent = 0);
    ~MoveRobot();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MoveRobot *m_ui;

    /**
      * Creates a new gridLayout inside a tabWidget with the robot name
      */
    QGridLayout* newGridLayoutForRobot(Robot* ptrRob);

    /**
      * Inisialize the slider associated with a Robot
      */
    void initSliders(QGridLayout *myGrid , Robot* ptrRob );

    /**
      * Makes a slider with a spinbox and a label
      */
    DofSlider* makeSlider(QGridLayout *myGrid, Robot* ptrRobot, p3d_jnt *Jnt,int DofNumOnJnt);

    /**
      * Sets the slider value to the Robot config
      */
    void setSliders(std::tr1::shared_ptr<Configuration> ptrConf);

    /**
      * Members
      */
    std::vector< std::vector<DofSlider*> >   mSliders;
    QTabWidget*                              mTabWidget;
};

#endif // MOVEROBOT_HPP
