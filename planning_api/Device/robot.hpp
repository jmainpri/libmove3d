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

    p3d_traj* getTrajStruct() {return _Robot->tcur;}

    /**
     * obtient le nom du Robot
     * @return le nom du Robot
     */
    std::string getName();

    /**
     * obtient un Graph calculé pour ce Robot
     * @param s le nom du Graph
     * @return le Graph ou un nouveau Graph si le Graph recherché n'existe pas
     */
    Graph* getGraph(std::string s);
    /**
     * obtient un Graph calculé pour ce Robot
     * @param i l'index du Graph dans le vecteur des Graph
     * @return le Graph ou un nouveau Graph si le Graph recherché n'existe pas
     */
    Graph* getGraph(int i);
    /**
     * obtient le Graph actif
     * @return le Graph actif
     */
    Graph* getActivGraph();
    /**
     * obtient le numéro du Graph actif
     * @return le numéro du Graph actif
     */
    int getActivGraphNumber();
    /**
     * modifie le Graph actif
     * @param S le nom du nouveau Graph actif
     */
    void setActivGraph(std::string S);
    /**
     * modifie le Graph actif
     * @param i l'index du nouveau Graph actif
     */
    void setActivGraph(int i);
    /**
     * obtient le nombre de Graph créés pour ce Robot
     * @return le nombre de Graph créés pour ce Robot
     */
    int getNbCreatedGraph();
    /**
     * ajoute un nouveau Graph au vecteur des Graph et le place comme Graph actif
     * @return le nouveau Graph
     */
    Graph* newGraph();
    /**
     * obtient le nombre de Graph
     * @return le nombre de Graph
     */
    int nbGraph();
    /**
     * insert un Graph à la fin du vecteur des Graph
     * @param G le Graph a insérer
     */
    void insertGraph(Graph* G);
    /**
     * retire un Graph du vecteur des Graph
     * @param num l'index du Graph à retirer
     */
    void removeGraph(int num);

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
    std::vector<Graph*> _Graph; /*!< le vecteur des Graph calculés pour ce Robot*/
    int activ_graph; /*!< le numéro du Graph actif*/
    int _nbCreatedGraph; /*!< le nombre total de Graph créés pour ce Robot*/

};

#endif
