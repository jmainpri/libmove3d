#ifndef WORKSPACE_HPP
#define WORKSPACE_HPP

#include "planningAPI.hpp"

/**
  * @ingroup NEW_CPP_MODULE
  * @defgroup CPP_API C++ Planning API
  * @brief Implements in C++ an interface to the more low level functionalities
  */

class Scene;

/**
 * @ingroup CPP_API
 * \brief Classe représentant l'espace de travail de l'application
 * @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */
class Project {

public:
	/**
	 * Class constructor
	 */
        Project();

        Project(std::string nameMainEnv);

	/**
	 * Class Destructor
	 */
        ~Project();

	/**
         * obtient l'Scene actif
         * @return l'Scene actif
	 */
        Scene* getActivScene();
	/**
         * modifie l'Scene actif
         * @param name le nom du nouvel Scene actif
	 */
        void setActivScene(std::string name);

	/**
         * insert un nouvel Scene au vecteur des Scene
         * @param E le nouvel Scene
	 */
        void insertScene(Scene* E);

private:
        std::vector<Scene*> _Scenes;/*!< le vecteur des Scene chargés*/
        std::string _activScene;/*!< le nom de l'Scene actif*/
};

extern Project WS;

#endif
