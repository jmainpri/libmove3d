#ifndef MOVEROBOT_HPP
#define MOVEROBOT_HPP

#include "qtLibrary.h"
#include "qtBase/SpinBoxSliderConnector_p.hpp"

#ifdef CXX_PLANNER
#include "planner_cxx/API/planningAPI.hpp"
#endif

#include "qtOpenGL/glwidget.hpp"

namespace Ui {
    class MoveRobot;
}

/**--------------------------------------------------------------
  * @ingroup qtMainWindow
  * @brief Creates one DoF slider structure
  */
class DofSlider : public QObject
{
    Q_OBJECT

public:
    DofSlider() {}
	
	DofSlider(Robot* R, GLWidget* Gl) : 
		mRobot(R),
		mOpenGl(Gl)
	{}
	
    ~DofSlider() {}
	
	/**
	 * Creates a slider with a spinbox and a label
	 */
    void makeSlider(QGridLayout* gridLayout, p3d_jnt *jntPt, int DofNumOnJnt);

    void setValue(double value) { mConnector->setValue(value); }
	
    QDoubleSpinBox* getDoubleSpinBox() { return mDoubleSpinBox;}
    QSlider* getHorizontalSlider() { return mHorizontalSlider; }
	
	QtShiva::SpinBoxSliderConnector* getConnector() {return mConnector;}
	
	
#ifdef CXX_PLANNER
    Robot* getRobot() { return mRobot; }
#endif

public slots:
    void dofValueChanged(double value);

private:
#ifdef CXX_PLANNER
    Robot*  mRobot;
#endif
	
    int					mDofNum;
	GLWidget*			mOpenGl;
	
	QLabel*				mLabel;
    QDoubleSpinBox*		mDoubleSpinBox;
    QSlider*			mHorizontalSlider;
	
	QtShiva::SpinBoxSliderConnector* mConnector;
};

/**--------------------------------------------------------------
 * @ingroup qtMainWindow
 * @brief Creates the sliders structure contains a vector of DofSlider
 */
class FormRobot : public QObject {
    
	Q_OBJECT
	
public:
	FormRobot() {}
	
	FormRobot(Robot* R,QGridLayout* GL,QComboBox* CB,GLWidget* openGl) :
		mRobot(R),
		mGridLayout(GL),
		mPositions(CB),
		mOpenGl(openGl)
	{}
	
	~FormRobot() {};
	
	/**
	 * Initializes the sliders associated to the Dofs of ptrRob
	 */
    void initSliders();
	
    /**
	 * Sets the associated sliders to the values int ptrConf
	 */
    void setSliders(Configuration& ptrConf);
	
	/**
	 * Resets the constrained DoFs
	 */
	void resetConstraintedDoFs();
	
	/**
	 * Returns the robot structure
	 */
	Robot* getRobot() { return mRobot; }
	
	/**
	 *
	 */
	QComboBox* getComboBox() { return mPositions; }
	
public slots:
	/**
	 * Sets the current Configuration to Init or Goto
	 */
	void setCurrentPosition(int position);
	
	/**
	 * Save current configuration to position (Init or GoTo)
	 */
	void saveCurrentConfigToPosition();
	
private:
	
    int calc_real_dof(void);
	
	Robot*						mRobot;
	std::vector<DofSlider*>		mSliders;
	QGridLayout*				mGridLayout;
	QComboBox*					mPositions;
	GLWidget*					mOpenGl;
};


/**--------------------------------------------------------------
  * @ingroup qtMainWindow
  * @brief Creates the FormRobots Stucture, Contains a vector of FormRobot
  */
class MoveRobot : public QWidget 
{
    Q_OBJECT
	
public:
    MoveRobot(QWidget *parent = 0);
    ~MoveRobot();
	
	/**
	 * Initilizes all forms
	 */
	void initAllForms(GLWidget* ptrOpenGl);
	
	/**
	 * Updates all robot pos
	 */
	void updateAllRobotInitPos();
	
	/**
	 * Get the robot form by names
	 */
	FormRobot* getRobotFormByName(std::string name);
	
	/**
	 * Sets the constraints
	 */
	void setRobotConstraintedDof(Robot* ptrRob);

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MoveRobot *m_ui;

#ifdef CXX_PLANNER
    /**
      * Creates a new gridLayout inside a tabWidget
      */
    FormRobot* newGridLayoutForRobot(Robot* ptrRob);
#endif

    /**
      * Members
      */
    std::vector<FormRobot*>		mRobots;
    QTabWidget*                 mTabWidget;
	GLWidget*					mOpenGl;
};

#endif // MOVEROBOT_HPP
