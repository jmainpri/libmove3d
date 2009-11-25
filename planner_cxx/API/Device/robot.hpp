#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "../planningAPI.hpp"

class Configuration;
class Graph;

/**
	\brief Classe représentant un Robot
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Robot{

public:
  //constructor and destructor
    /**
     * Constructeur de la classe
     * @param R le p3d_rob pour lequel l'objet Robot est créé
     */
    Robot(p3d_rob* R);

    /**
     * Destructeur de la classe
     */
    ~Robot();

  //Accessor
    /**
     * obtient la structure p3d_rob de la classe
     * @return la structure p3d_rob
     */
    p3d_rob* getRobotStruct();

    /**
     * Gets traj
     * @return pointer to structure p3d_traj
     */
    p3d_traj* getTrajStruct() {return _Robot->tcur;}

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
    std::tr1::shared_ptr<Configuration> shoot(bool samplePassive = true);
    /**
     * obtient une Configuration-Direction aléatoire pour le Robot
     * @param samplePassive (default = true) indique si l'on tire les joints passif ou non (ie. FALSE dans le cas de ML-RRT)
     * @return la Configuration tirée
     */
    std::tr1::shared_ptr<Configuration> shootDir(bool samplePassive = true);

    /**
     * place le Robot dans une Configuration
     * @param q la Configuration dans laquelle le Robot sera placé
     * @return la Configuration est atteignable cinématiquement
     */
    int setAndUpdate(Configuration& q);

    /**
     * obtient la Configuration current du Robot
     * @return la Configuration current du Robot
     */
    std::tr1::shared_ptr<Configuration> getInitialPosition();
    /**
     * obtient la Configuration GoTo du Robot
     * @return la Configuration GoTo du Robot
     */
    std::tr1::shared_ptr<Configuration> getGoTo();

    /**
     *
     */
    std::tr1::shared_ptr<Configuration> getCurrentPos();

    /**
     *
     */
    std::vector<double> getJointPos(int id);

private:
    p3d_rob* _Robot; /*!< une structure de p3d_rob contenant les données sur le Robot*/
    std::string _Name; /*!< le nom du Robot*/

};

#endif
