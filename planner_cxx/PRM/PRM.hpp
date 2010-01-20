#ifndef PRM_HPP
#define PRM_HPP

#include "../planner.hpp"
/**
    @ingroup NEW_CPP_MODULE
    @defgroup PRM Probabilistic-RM
    @brief Probacilistic roadmap module
    \image html prm.jpg
*/

/**
    @ingroup PRM
    @brief Classe représentant l'algorithme PRM
    @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class PRM : public Planner
{
public:
    /**
     * Constructeur de la classe
     * @param WS Le WorkSpace de l'application
     */
    PRM(Robot* R, Graph* G);

    /**
     * Destructeur de la classe
     */
    ~PRM();

    /**
     * initialise le Planner
     * @return le nombre de Node ajoutés lors de l'initialisation
     */
    virtual int init();

    /**
     * test les conditions d'arret
     * @param (*fct_stop)(void) la fonction d'arret
     * @return l'algorithme doit s'arreter
     */
    bool checkStopConditions();

    /**
     * fonction principale de l'algorithme PRM
     * @param Graph_Pt le graphPt affiché
     * @param (*fct_stop)(void) la fonction d'arret
     * @param (*fct_draw)(void) la fonction d'affichage
     * @return le nombre de Node ajoutés au Graph
     */
    uint expand();

protected:
    int _nbConscutiveFailures; /*!< nombre d'échecs consécutifs*/
};

#endif
