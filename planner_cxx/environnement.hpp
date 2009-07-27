#ifndef ENVIRONNEMENT_HPP
#define ENVIRONNEMENT_HPP

#include "../planning_api/planningAPI.hpp"

/**
	\brief Classe représentant un Environnement de travail (ie. un fichier .p3d)
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Environnement{
private:
    std::vector<Robot*> _Robots;/*!< le vecteur des Robot de l'Environnement*/
    std::string _Name;/*!< le nom de l'Environnement*/
    std::string _activRobot;/*!< le nom du Robot actif*/

public:
    /**
     * Constructeur de la classe
     * @param name le nom de l'Environnement
     */
    Environnement(std::string name);

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

};

#endif
