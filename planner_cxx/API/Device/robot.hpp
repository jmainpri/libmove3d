#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "../planningAPI.hpp"

class Configuration;
class Graph;

#ifndef _DEVICE_H
typedef struct p3d_rob;
typedef struct p3d_traj;
#endif
/**
  * @ingroup CPP_API
  * @defgroup ROBOT Device
  * @brief The robot class
  */


/**
        @ingroup ROBOT
        @brief Classe représentant un Robot
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Robot{

public:
  //constructor and destructor
    /**
     * Constructeur de la classe
     * @param R le p3d_rob pour lequel l'objet Robot est créé
     */
    Robot(p3d_rob* R , bool copy = false );

    /**
     * Destructeur de la classe
     */
    ~Robot();

    p3d_rob* copyRobotStruct(p3d_rob* robotPt);

  //Accessor
    /**
     * obtient la structure p3d_rob de la classe
     * @return la structure p3d_rob
     */
    p3d_rob* getRobotStruct();

    /**
     * Gets traj associated with Robot
     * @return pointer to structure p3d_traj
     */
    p3d_traj* getTrajStruct();

    /**
     * obtient le nom du Robot
     * @return le nom du Robot
     */
    std::string getName();

    /**
     * tire une Configuration aléatoire pour le Robot
     * @param samplePassive (default = TRUE) indique si l'on tire les joints passif ou non (ie. FALSE dans le cas de ML-RRT)
     * @return la Configuration tirée
     */
    std::tr1::shared_ptr<Configuration> shoot(bool samplePassive = false);
    /**
     * obtient une Configuration-Direction aléatoire pour le Robot
     * @param samplePassive (default = true) indique si l'on tire les joints passif ou non (ie. FALSE dans le cas de ML-RRT)
     * @return la Configuration tirée
     */
    std::tr1::shared_ptr<Configuration> shootDir(bool samplePassive = false);

    /**
      * Gets the active free flyer inside a box
      */
    std::tr1::shared_ptr<Configuration> shootFreeFlyer(double* box);

    /**
     * place le Robot dans une Configuration
     * @param q la Configuration dans laquelle le Robot sera placé
     * @return la Configuration est atteignable cinématiquement
     */
    int setAndUpdate(Configuration& q);

    /**
     * place le Robot dans une Configuration, without checking the cinematic constraints.
     * @param q la Configuration dans laquelle le Robot sera placé
     */
     void setAndUpdateWithoutConstraints(Configuration& q);

    /**
     * obtient la Configuration current du Robot
     * @return la Configuration current du Robot
     */
    std::tr1::shared_ptr<Configuration> getInitialPosition();
	
	/**
	 * Sets the Initial Position of the Robot
	 */
	void setInitialPosition(Configuration& conf);
	
    /**
     * obtient la Configuration GoTo du Robot
     * @return la Configuration GoTo du Robot
     */
    std::tr1::shared_ptr<Configuration> getGoTo();
	
	/**
	 * Sets the Goto Position of the Robot
	 */
	void setGoTo(Configuration& conf);

    /**
     *
     */
    std::tr1::shared_ptr<Configuration> getCurrentPos();
	
    /**
      *
      */
    std::tr1::shared_ptr<Configuration> getNewConfig();

    /**
     *
     */
    Vector3d getJointPos(int id);

    /**
      *
      */
    int getObjectDof() 
	{ 
#ifdef LIGHT_PLANNER
		return _Robot->curObjectJnt->index_dof; 
#else
		std::cout << "Warning: no light planner" << std::endl;
		return 0;
#endif
	}

private:
    p3d_rob* _Robot; /*!< une structure de p3d_rob contenant les données sur le Robot*/
    std::string _Name; /*!< le nom du Robot*/
	bool _copy;

};

#endif
