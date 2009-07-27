#ifndef WORKSPACE_HPP
#define WORKSPACE_HPP

#include "environnement.hpp"

/**
	\brief Classe représentant l'espace de travail de l'application
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class WorkSpace{
private:
	std::vector<Environnement*> _Environnements;/*!< le vecteur des Environnement chargés*/
	std::string _activEnvironnement;/*!< le nom de l'Environnement actif*/

public:
    /**
     * Constructeur de la classe
     */
    WorkSpace();

    /**
     * Destructeur de la classe
     */
    ~WorkSpace();

    /**
     * obtient l'Environnement actif
     * @return l'Environnement actif
     */
    Environnement* getActivEnvironnement();
    /**
     * modifie l'Environnement actif
     * @param name le nom du nouvel Environnement actif
     */
    void setActivEnvironnement(std::string name);

    /**
     * insert un nouvel Environnement au vecteur des Environnement
     * @param E le nouvel Environnement
     */
    void insertEnvironnement(Environnement* E);
};

extern WorkSpace WS;

#endif
