#ifndef MOVEROBOT_HPP
#define MOVEROBOT_HPP

#include "../qtLibrary.h"
#include "../qtBase/SpinBoxSliderConnector_p.hpp"
#include "../../planner_cxx/API/planningAPI.hpp"

namespace Ui {
    class MoveRobot;
}

/**
  * @ingroup qtMainWindow
  * @brief Creates one DoF slider structure
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
  * @brief Creates the sliders structure
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
      * Creates a new gridLayout inside a tabWidget
      */
    QGridLayout* newGridLayoutForRobot(Robot* ptrRob);

    /**
      * Initializes the sliders associated to the Dofs of ptrRob
      */
    void initSliders(QGridLayout *myGrid , Robot* ptrRob );

    /**
      * Creates a slider with a spinbox and a label
      */
    DofSlider* makeSlider(QGridLayout *myGrid, Robot* ptrRobot, p3d_jnt *Jnt,int DofNumOnJnt);

    /**
      * Sets the associated sliders to the values int ptrConf
      */
    void setSliders(std::tr1::shared_ptr<Configuration> ptrConf);

    int calc_real_dof(void);

    /**
      * Members
      */
    std::vector< std::vector<DofSlider*> >   mSliders;
    QTabWidget*                              mTabWidget;
};

#endif // MOVEROBOT_HPP
