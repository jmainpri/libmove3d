#ifndef __MANIPULATIONUTILS_HPP__
#define __MANIPULATIONUTILS_HPP__

#include "ManipulationStruct.h"
#ifdef GRASP_PLANNING
  #include "GraspPlanning-pkg.h"
#endif
#include "P3d-pkg.h"

/** @defgroup manipulation
* The manipulation classes are dedicated to
* the planning of manipulation tasks.
* They encapsulates variables and functions that allow to easily (!) plan the motion
* required for basic manipulation tasks (pick-and-place, object transfer from one hand to the other).
 */

//! @ingroup manipulation
/** Different general utils for manipulation*/
class  ManipulationUtils {
  public:
    /*Constructors and Destructors*/

    ManipulationUtils(){};
    virtual ~ManipulationUtils(){};


    /*############# Static Functions ############*/

    /* Message gestion */
    static void undefinedRobotMessage();
    static void undefinedObjectMessage();
    static void undefinedSupportMessage();
    static void undefinedCameraMessage();
    static void printManipulationMessage(MANIPULATION_TASK_MESSAGE message);
    static void printManipulationError(MANIPULATION_TASK_MESSAGE message);
    static int printConstraintInfo(p3d_rob* robot);
    
    /* UI gestion */
    //! Forbids all the interaction (keyboard and mouse) with the current window.
    //! \return 0 in case of success, 1 otherwise
    static int forbidWindowEvents();
    
    //! Allows the interaction (keyboard and mouse) with the current window.
    //! \return 0 in case of success, 1 otherwise
    static int allowWindowEvents();

    //! Copy the given configuration to the robot XForm window
    //! \return 0 in case of success, 1 otherwise
    static int copyConfigToFORM(p3d_rob* robot, configPt q);
};

//! @ingroup manipulation
/** This Class contains all necessessary data to specify a arm to manipulate with it*/
class ArmManipulationData {
		
  public:
    ArmManipulationData(p3d_cntrt* ccCntrt = NULL, p3d_cntrt* fkCntrt = NULL, p3d_jnt *manipulationJnt = NULL, int cartesianGroup = -1, int cartesianSmGroup = -1, int handGroup = -1, int handSmGroup = -1){
			_manipState = handFree;
      _ccCntrt = ccCntrt;
      _fkCntrt = fkCntrt;
      _manipulationJnt = manipulationJnt;
#ifdef MULTILOCALPATH
      _cartesianGroup = cartesianGroup;
      _cartesianSmGroup = cartesianSmGroup;
      _handGroup = handGroup;
      _handSmGroup = handSmGroup;
#endif
      _carriedObject = NULL;
      _placement = NULL;
      _human = NULL;
      _cartesian = false;
    };

    virtual ~ArmManipulationData(){};

    /*************/
    /* Functions */
    /*************/
    /** Fix hand dof, Disable autocollisions, and set the arm to rest configuration*/
    void fixHand(p3d_rob* robot, bool rest);
    /** Unfix hand dof, Enable autocollisions*/
    void unFixHand(p3d_rob* robot);


    /***********/
    /* Setters */
    /***********/
    inline void setCcCntrt(p3d_cntrt* ccCntrt){
      _ccCntrt = ccCntrt;
    };
    inline void setCcCntrt(p3d_rob* robot, int id){
      _ccCntrt = robot->cntrt_manager->cntrts[id];
    };
    inline void setFkCntrt(p3d_cntrt* fkCntrt){
      _fkCntrt = fkCntrt;
    };
    inline void setFkCntrt(p3d_rob* robot, int id){
      _fkCntrt = robot->cntrt_manager->cntrts[id];
    };
    inline void setManipulationJnt(p3d_jnt* manipulationJnt){
      _manipulationJnt = manipulationJnt;
    };
    inline void setManipulationJnt(p3d_rob* robot, int manipulationJnt){
      _manipulationJnt = robot->joints[manipulationJnt];
    };
#ifdef MULTILOCALPATH
    inline void setCartesianGroup(int cartesianGroup){
      _cartesianGroup = cartesianGroup;
    };
    inline void setCartesianSmGroup(int cartesianSmGroup){
      _cartesianSmGroup = cartesianSmGroup;
    };
    inline void setHandGroup(int handGroup){
      _handGroup = handGroup;
    };
    inline void setHandSmGroup(int handSmGroup){
      _handSmGroup = handSmGroup;
    };
#endif
		inline void setCarriedObject(p3d_rob* carriedObject){
			_carriedObject = carriedObject;
		};
		inline void setCarriedObject(const char* robotName){
			_carriedObject = p3d_get_robot_by_name(robotName);
		};
    inline void setPlacement(p3d_rob* placement){
      _placement = placement;
    };
    inline void setHuman(p3d_rob* human){
      _human = human;
    };
#ifdef GRASP_PLANNING
    inline void setHandProperties(int handId){
      _handProp.initialize((gpHand_type)handId);
    };
#endif
    inline void setCartesian(bool cartesian){
      cartesian = _cartesian;
    };

    /***********/
    /* Getters */
    /***********/

    inline p3d_cntrt* getCcCntrt(void) const{
      return _ccCntrt;
    };
    inline p3d_cntrt* getFkCntrt(void) const{
      return _fkCntrt;
    };
    inline p3d_jnt* getManipulationJnt(void) const{
      return _manipulationJnt;
    };
#ifdef MULTILOCALPATH
    inline int getCartesianGroup(void) const{
      return _cartesianGroup;
    };
    inline int getCartesianSmGroup(void) const{
      return _cartesianSmGroup;
    };
    inline int getHandGroup(void) const{
      return _handGroup;
    };
    inline int getHandSmGroup(void) const{
      return _handSmGroup;
    };
#endif
    inline p3d_rob* getCarriedObject(void) const{
      return _carriedObject;
    };
    inline p3d_rob* getPlacement(void) const{
      return _placement;
    };
    inline p3d_rob* getHuman(void) const{
      return _human;
    };
#ifdef GRASP_PLANNING
    inline gpHand_properties getHandProperties() const{
      return _handProp;
    };
#endif
    inline bool getCartesian(void) const{
      return _cartesian;
    };
	
	MANIPULATION_ARM_STATE& getManipState() {
		return _manipState;
	}
	
	private :
	//! Arm object-manipulation state
	//! The state should be provided by an external module
	MANIPULATION_ARM_STATE _manipState;
	
	/***************/
	/* Constraints */
	/***************/
	/** Arm associated Closed Chain Constraint*/
	p3d_cntrt * _ccCntrt;
	/** Arm corresopnding Forward kinematic Constraint*/
	p3d_cntrt * _fkCntrt;
	/** Freeflyer */
	p3d_jnt * _manipulationJnt;
	/** < choose to plan the arm motion in cartesian space (for the end effector) or joint space  */
	bool _cartesian;
	
#ifdef MULTILOCALPATH
	/******************/
	/* Multilocalpath */
	/******************/
	/** MultiLocal Path cartesian Linear Group id*/
	int _cartesianGroup;
	/** MultiLocal Path cartesian SoftMotion Group id*/
	int _cartesianSmGroup;
	/** MultiLocal Path hand Linear Group id*/
	int _handGroup;
	/** MultiLocal Path hand SoftMotion Group id*/
	int _handSmGroup;
#endif
	
	/************************/
	/* Manipulation Objects */
	/************************/
	/** The object carried by this arm*/
	p3d_rob* _carriedObject;
	/** The object where to place the object carried by this arm*/
	p3d_rob* _placement;
	/** The human to interract with with this arm*/
	p3d_rob* _human;
	
#ifdef GRASP_PLANNING
	/************/
	/* Grasping */
	/************/
	/** Arm end effector property */
	gpHand_properties _handProp;
#endif
	
};

#ifdef GRASP_PLANNING
//! @ingroup manipulation
//! this class holds the manipulation data
//! hence every intermediary configuration that is used 
//! to plan a manipulation problem
class ManipulationData{
  public:
    ManipulationData(p3d_rob* robot){
      _robot = robot;
      _grasp = NULL;
      _graspConfig = p3d_alloc_config(robot);
      _graspConfigCost = P3D_HUGE;
      _openConfig = p3d_alloc_config(robot);
      _approachFreeConfig = p3d_alloc_config(robot);
      _approachGraspConfig = p3d_alloc_config(robot);
      p3d_mat4Copy(p3d_mat4IDENTITY ,_graspAttachFrame);
    };
    ManipulationData(p3d_rob* robot, gpGrasp* grasp, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, configPt approachGraspConfig, p3d_matrix4 graspAttachFrame){
      _robot = robot;
      _grasp = grasp;
      _graspConfig = graspConfig;
      _graspConfigCost = P3D_HUGE;
      _openConfig = openConfig;
      _approachFreeConfig = approachFreeConfig;
      _approachGraspConfig = approachGraspConfig;
      p3d_mat4Copy(graspAttachFrame ,_graspAttachFrame);
    };
    virtual ~ManipulationData(){
      if(_graspConfig){
        p3d_destroy_config(_robot, _graspConfig);
        _graspConfig = NULL;
      }
      if(_openConfig){
        p3d_destroy_config(_robot, _openConfig);
        _openConfig = NULL;
      }
      if(_approachFreeConfig){
        p3d_destroy_config(_robot, _approachFreeConfig);
        _approachFreeConfig = NULL;
      }
      if(_approachGraspConfig){
        p3d_destroy_config(_robot, _approachGraspConfig);
        _approachGraspConfig = NULL;
      }
      if(_grasp){
        delete(_grasp);
        _grasp = NULL;
      }
    }
    //Getters
    inline p3d_rob* getRobot() const{
      return _robot;
    }
    inline gpGrasp* getGrasp() const{
      return _grasp;
    }
    inline configPt getGraspConfig() const{
      return _graspConfig;
    }
    inline configPt getOpenConfig() const{
      return _openConfig;
    }
    inline configPt getApproachFreeConfig() const{
      return _approachFreeConfig;
    }
    inline configPt getApproachGraspConfig() const{
      return _approachGraspConfig;
    }
    inline void getAttachFrame(p3d_matrix4 graspAttachFrame) const{
      p3d_mat4Copy((p3d_matrix_type(*)[4])_graspAttachFrame, graspAttachFrame);
    }
    inline double getGraspConfigCost() const{
      return _graspConfigCost;
    }
    //Setters
    inline void setGrasp(gpGrasp* grasp){
			if (!grasp) {
				_grasp = NULL;
				return;
			}
      if(_grasp){
        delete(_grasp);
        _grasp = NULL;
      }
      _grasp = new gpGrasp(*grasp);
    }
    inline void setGraspConfig(configPt graspConfig){
      p3d_copy_config_into(_robot, graspConfig, &_graspConfig);
    }
    inline void setOpenConfig(configPt openConfig){
      p3d_copy_config_into(_robot, openConfig, &_openConfig);
    }
    inline void setApproachFreeConfig(configPt approachFreeConfig){
      p3d_copy_config_into(_robot, approachFreeConfig, &_approachFreeConfig);
    }
    inline void setApproachGraspConfig(configPt approachGraspConfig){
      p3d_copy_config_into(_robot, approachGraspConfig, &_approachGraspConfig);
    }
    inline void setAttachFrame(p3d_matrix4 graspAttachFrame){
      p3d_mat4Copy(graspAttachFrame, _graspAttachFrame);
    }
    inline void setGraspConfigCost(double graspConfigCost){
      _graspConfigCost = graspConfigCost;
    }
    ManipulationData& operator = (const ManipulationData& data){
      _robot = data.getRobot();
      setGrasp(data.getGrasp());
      setGraspConfig(data.getGraspConfig());
      setOpenConfig(data.getOpenConfig());
      setApproachFreeConfig(data.getApproachFreeConfig());
      setApproachGraspConfig(data.getApproachGraspConfig());
      p3d_matrix4 att;
      data.getAttachFrame(att);
      setAttachFrame(att);
      setGraspConfigCost(data.getGraspConfigCost());
      return *this;
    }
  private:
    p3d_rob* _robot;
    gpGrasp* _grasp;
    configPt _graspConfig;
    configPt _openConfig;
    configPt _approachFreeConfig;
    configPt _approachGraspConfig;
    double _graspConfigCost;
    p3d_matrix4 _graspAttachFrame;
};
#endif
#endif
