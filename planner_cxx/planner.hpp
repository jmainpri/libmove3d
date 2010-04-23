#ifndef PLANNER_HPP
#define PLANNER_HPP

#include "API/planningAPI.hpp"
/**
  * @defgroup NEW_CPP_MODULE C++ Module
  * This Module takes in all that has been done with the new C++ API
  */

/**
 * @ingroup NEW_CPP_MODULE
 * @brief Base class for planning algorithms
 * @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Planner {

public:
    /**
     * Plain Constructor of the class
     */
	Planner();
	
    /**
     * Constructor of the class
     *
     * @param rob a robot robot
     * @param graph a graph belonging to that robot
     */
    Planner(Robot* rob, Graph* graph);

    /**
     * Destructeur de la classe
     */
    virtual ~Planner();

    /**
     * test de trajectoire
     * @return la trajectoire entre les Node Start et Goal existe
     */
    bool trajFound();

    /**
     * retourne le Robot activ
     * @return Le Robot activ
     */
    Robot* getActivRobot();

    /**
     * place le Robot utilisé pour la planification
     * @param R le Robot pour lequel la planification va se faire
     */
    void setRobot(Robot* R);

    /**
     * obtient le Graph actif pour la planification
     * @return le Graph actif pour la planification
     */
    Graph* getActivGraph();
	
    /**
     * modifie le Graph actif pour la planification
     * @param G le nouveau Graph activ
     */
    void setGraph(Graph* G);

    /**
     * place le Node initial de la planification
     * @param Cs la Configuration initiale du robot pour la planification
     * @return un Node a été ajouté au graph
     */
    bool setStart(std::tr1::shared_ptr<Configuration> Cs);
	
    /**
     * place le Node final de la planification
     * @param Cg la Configuration finale du robot pour la planification
     * @return un Node a été ajouté au graph
     */
    bool setGoal(std::tr1::shared_ptr<Configuration> Cg);

    /**
     * obtient le Node intial de la planification
     * @return le Node intial de la planification
     */
    Node* getStart();
	
    /**
     * obtient le Node final de la planification
     * @return le Node final de la planification
     */
    Node* getGoal();
    
	/**
     * test si le Planner est initialisé pour la planification
     * @return le Planner est initialisé
     */
    bool getInit();
    
	/**
     * modifie la valeur du Booleen de test d'initialisation
     * @param b la valeur entrée
     */
    void setInit(bool b);

    /**
     * Méthode d'initialisation du Planner
     */
    virtual int init();

protected:
    int (*_stop_func)();
    void (*_draw_func)();

    Node* _Start; /*!< Le Node initial de la planification*/
    Node* _Goal; /*!< Le Node final de la planification*/

    Robot* _Robot;/*!< Le Robot pour lequel la recherche va se faire*/
    Graph* _Graph;/*!< Le Graph qui va être utilisé*/

    bool _Init;/*!< Le Planner a été initialisé*/

};

#endif
