#ifndef __MANIPULATIONUTILS_HPP__
#define __MANIPULATIONUTILS_HPP__

#include "ManipulationStruct.h"
#include "GraspPlanning-pkg.h"
#include "P3d-pkg.h"

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

    /* UI gestion */
    //! Forbids all the interaction (keyboard and mouse) with the current window.
    //! \return 0 in case of success, 1 otherwise
    static int forbidWindowEvents();
    
    //! Allows the interaction (keyboard and mouse) with the current window.
    //! \return 0 in case of success, 1 otherwise
    static int allowWindowEvents();
};

//! @ingroup manipulation
/** This Class contains all necessessary data to specify a arm to manipulate with it*/
class ArmManipulationData {
  private :
     /***************/
     /* Constraints */
     /***************/
    /** Arm associated Closed Chain Constraint*/
    p3d_cntrt * _ccCntrt;
    /** Arm corresopnding Forward kinematic Constraint*/
    p3d_cntrt * _fkCntrt;
    /** Arm corresopnding Forward kinematic Constraint*/
    p3d_jnt * _manipulationJnt;
    /** < choose to plan the arm motion in cartesian space (for the end effector) or joint space  */
    bool _cartesian;
     /******************/
     /* Multilocalpath */
     /******************/
    /** MultiLocal Path cartesian Linear Group id*/
    int _cartesianGroup;
    /** MultiLocal Path cartesian SoftMotion Group id*/
    int _cartesianSmGroup;

    /************************/
    /* Manipulation Objects */
    /************************/
    /** The object carried by this arm*/
    p3d_rob* _carriedObject;
    /** The object where to place the object carried by this arm*/
    p3d_rob* _placement;
    /** The human to interract with with this arm*/
    p3d_rob* _human;

    /************/
    /* Grasping */
    /************/
    /** Arm end effector property */
    gpHand_properties _handProp;

  public:
    ArmManipulationData(p3d_cntrt* ccCntrt = NULL, p3d_cntrt* fkCntrt = NULL, p3d_jnt *manipulationJnt = NULL, int cartesianGroup = -1, int cartesianSmGroup = -1){
      _ccCntrt = ccCntrt;
      _fkCntrt = fkCntrt;
      _manipulationJnt = manipulationJnt;
      _cartesianGroup = cartesianGroup;
      _cartesianSmGroup = cartesianSmGroup;
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
    void fixHand(p3d_rob* robot);
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
    inline void setCartesianGroup(int cartesianGroup){
      _cartesianGroup = cartesianGroup;
    };
    inline void setCartesianSmGroup(int cartesianSmGroup){
      _cartesianSmGroup = cartesianSmGroup;
    };
    inline void setCarriedObject(p3d_rob* carriedObject){
      _carriedObject = carriedObject;
    };
    inline void setPlacement(p3d_rob* placement){
      _placement = placement;
    };
    inline void setHuman(p3d_rob* human){
      _human = human;
    };
    inline void setHandProperties(int handId){
      _handProp.initialize((gpHand_type)handId);
    };
    inline void setCartesian(bool cartesian){
      cartesian = _cartesian;
    };

    /***********/
    /* Getters */
    /***********/

    inline p3d_cntrt* getCcCntrt(void){
      return _ccCntrt;
    };
    inline p3d_cntrt* getFkCntrt(void){
      return _fkCntrt;
    };
    inline p3d_jnt* getManipulationJnt(void){
      return _manipulationJnt;
    };
    inline int getCartesianGroup(void){
      return _cartesianGroup;
    };
    inline int getCartesianSmGroup(void){
      return _cartesianSmGroup;
    };
    inline p3d_rob* getCarriedObject(void){
      return _carriedObject;
    };
    inline p3d_rob* getPlacement(void){
      return _placement;
    };
    inline p3d_rob* getHuman(void){
      return _human;
    };
    inline gpHand_properties getHandProperties(){
      return _handProp;
    };
    inline bool getCartesian(void){
      return _cartesian;
    };
};

#endif
