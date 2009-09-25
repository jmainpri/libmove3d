#ifndef NODE_HPP
#define NODE_HPP

#include "../planningAPI.hpp"

/**
	\brief Classe représentant un Node d'un Graph
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Node{

public:

  //Constructor and destructor
	Node();
    /**
     * Constructeur de la classe
     * @param G le Graph pour lequel le Node est créé
     * @param C la Configuration stockée dans le Node
     */
    Node(Graph* G, std::tr1::shared_ptr<Configuration> C);
    /**
     * Constructeur de la classe
     * @param G le Graph pour lequelle Node est créé
     * @param N la structure de p3d_node qui sera stockée
     */
    Node(Graph* G, p3d_node* N);

    Node(const Node& N);

    /**
     * destructeur de la classe
     */
    ~Node();

    bool operator==(Node& N);

  //Accessors
    /**
     * obtient la structure p3d_node
     * @return la structure p3d_node
     */
    p3d_node* getNodeStruct();

    /**
     * obtient le Graph pour lequel le Node a été créé
     * @return le GRaph pour lequel le Node a été créé
     */
    Graph* getGraph();

    /**
     * obtient le Robot pour lequel le Node a été créé
     * @return le Robot pour lequel le Node a été créé
     */
    Robot* getRobot();

    /**
     * obtient la Configuration stockée
     * @return la Configuration stockée
     */
    std::tr1::shared_ptr<Configuration> getConfiguration();

    /**
     * modifie la valeur de activ
     * @param b la nouvelle valeur de activ
     */
    void activ(bool b);
    /**
     * obtient la valeur de activ
     * @return la valeur de activ
     */
    bool isActiv();

    /**
     * obtient le cout du Node
     * @return le cout du Node
     */
    double getCost();

    /**
     * Returns the Sum of cost along the path
     */
    double getSumCost();
    /**
     * obtient la temperature du Node
     * @return la temperature du Node
     */
    double getTemp() { return(_Node->temp); }
    /**
     * modifie la temperature du Node
     * @param t la nouvelle temperature
     */
    void setTemp(double t) { _Node->temp = t; }

    /**
      * Set node a Start Compco node
      */
    void setInStart() { _isInStartCompco = true; }
    /**
      * Get is Start Compco Node
      */
    bool getInStart() { return _isInStartCompco; }

    /**
      * Set Node e Goal Compco Node
      */
    bool setInGoal() { _isInGoalCompco = true; }
    /**
      * Get is Start Compco Node
      */
    bool getInGoal() { return _isInGoalCompco; }



    /**
     * obtient la distance au Node courant créé
     * @return la distance au Node courant créé
     */
    double getDist();
    /**
     * calcule la distance entre deux Node et stocke la valeur dans DistNew
     * @param N le Node à partir duquel on veut calculer la distance
     * @return la distance entre les deux Node
     */
    double dist(Node* N);

    /**
     * teste si deux Node sont égaux
     * @param N le Node à tester
     * @return les Node sont égaux
     */
    bool equal(Node* N);

    /**
     * teste si deux Node sont dans la même composante connexe
     * @param N le Node à tester
     * @return les deux Node sont dans la même composante connexe
     */
    bool inSameComponent(Node* N);

    /**
     * Get Number of neighbors
     */
    int getNumberOfNeighbors() { return _Node->nneighb; }

    /**
     * teste si deux Node peuvent être liés
     * @param N le Node à tester
     * @param dist la distance entre les Node
     * @return les Node peuvent être liés
     */
    bool isLinkable(Node* N, double* dist);

    /**
     * vérifie si le poids max du Node a été atteint
     */
    void checkStopByWeight();

    //fonctions portant sur les composantes connexes
    /**
     * obtient la structure de composante connexe à laquelle appartient le Node
     * @return la structure de composante connexe à laquelle appartient le Node
     */
    p3d_compco* getCompcoStruct();
    /**
     * obtient le pointeur sur la structure Compco
     * @return le pointeur sur la structure Compco
     */
    p3d_compco** getCompcoStructPt();

    /**
     * place le Node dans la composante connexe
     * @param compco la composante dans laquelle le Node doit être placé
     */
    void setCompco(p3d_compco* compco);

    /**
     * detruit la composante connexe
     */
    void deleteCompco();

    /**
     * teste si la composante connexe a atteint le nombre max de Nodes
     * @return la composante connexe a atteint le nombre max de Nodes
     */
    bool maximumNumberNodes();

    /**
     * connect un Node à la composante connexe
     * @param N le Node à connecter
     * @return le Node est connecté
     */
    bool connectNodeToCompco(Node* N, double step);

    /**
     * merge deux composantes connexes
     * @param compco la composante connexe à merger
     */
    void merge(Node* compco);
    /**
     * teste si deux composante connexe sont égales
     * @param compco la composante connexe à tester
     * @return les deux composantes sont égales
     */
    bool equalCompco(Node* compco);
    /**
     * tire un Node aléatoirement dans la composante connexe
     * @return le Node tiré
     */
    Node* randomNodeFromComp();

    void setSelectCost(double Cost) { _SelectCost = Cost; }

    double getSelectCost() { return _SelectCost; }

    void setExpandFailed() { _nbExpan++;  }

    int getNbExpandFailed() { return _nbExpan; }

    void print();

    std::vector<Node*>& getSortedNodes() {return _SortedNodes;}
    void setSortedNodes( std::vector<Node*>& nodes ) { _SortedNodes = nodes;}

private:

    p3d_node* _Node;
    Graph* _Graph;
    Robot* _Robot;

    std::tr1::shared_ptr<Configuration> _Configuration;
    bool _activ;

    bool _isInStartCompco;
    bool _isInGoalCompco;

    double _SelectCost;
    int _nbExpan;
    std::vector<Node*> _SortedNodes;

};

#endif
