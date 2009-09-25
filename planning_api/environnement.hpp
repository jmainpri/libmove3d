#ifndef ENVIRONNEMENT_HPP
#define ENVIRONNEMENT_HPP

#include "planningAPI.hpp"

/**
	\brief Class that represents a Environment,
	Described by a p3d file

	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Robot;

class Environnement{

public:
    /**
     * Constructeur de la classe
     * @param name le nom de l'Environnement
     */
    Environnement(std::string name);

    Environnement(std::string name, Robot* Robot);
    /**
     * Destructeur de la classe
     */
    ~Environnement();

    /**
     * obtient le nom de l'Environnement
     * @return le nom de l'Environnement
     */
    std::string getName();

    /**
     * modifie le Robot actif
     * @param name le nom du nouveau Robot actif
     */
    void setActivRobot(std::string name);
   /**
     * obtient le Robot actif
     * @return le Robot actif; NULL si le Robot ne peux pas être créé
     */
    Robot* getActivRobot();

    /**
     * insert un nouveau Robot au vecteur des Robot
     * @param R le nouveau Robot
     */
    void insertRobot(Robot* R);

private:
    std::vector<Robot*> _Robots;/*!< le vecteur des Robot de l'Environnement*/
    std::string _Name;/*!< le nom de l'Environnement*/
    std::string _activRobot;/*!< le nom du Robot actif*/

};

#endif
