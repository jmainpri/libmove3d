#ifndef ENVIRONNEMENT_HPP
#define ENVIRONNEMENT_HPP

#include "planningAPI.hpp"

class Robot;

/**
        @ingroup CPP_API

        @brief Class that represents a Scene,
        Described by a p3d file

        @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Scene{

public:
    /**
     * Constructeur de la classe
     * @param name le nom de l'Scene
     */
    Scene(std::string name);

    Scene(std::string name, Robot* Robot);
    /**
     * Destructeur de la classe
     */
    ~Scene();

    /**
     * obtient le nom de l'Scene
     * @return le nom de l'Scene
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
    std::vector<Robot*> _Robots;/*!< le vecteur des Robot de l'Scene*/
    std::string _Name;/*!< le nom de l'Scene*/
    std::string _activRobot;/*!< le nom du Robot actif*/

};

#endif
