#ifndef EDGE_HPP
#define EDGE_HPP

#include "../planningAPI.hpp"

/**
	\brief Classe représentant une Edge d'un Graph
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Edge{

private:
  p3d_edge* _Edge;
  Node* _Start;
  Node* _End;
  Graph* _Graph;
  Robot* _Robot;
  double _Long;


public:
  //contructor and destructor
    /**
     * Constructeur de la classe
     * @param G le Graph pour laquel l'Edge est créée
     * @param E la structure d'edge qui sera stockée
     */
    Edge(Graph* G, p3d_edge* E);
    /**
     * Constructeur de la classe
     * @param G le Graph pour lequel l'Edge est créée
     * @param N1 le Node initial de l'Edge
     * @param N2 le Node final de l'Edge
     * @param Long la longueur de l'Edge
     */
    Edge(Graph* G, Node* N1, Node* N2, double Long);

    /**
     * Destructeur de la classe
     */
    ~Edge();

  //Accessors
    /**
     * obtient la structure p3d_edge stockée
     * @return la structure p3d_edge stockée
     */
    p3d_edge* getEdgeStruct();

    /**
     * obtient le Graph pour lequel l'Edge est créée
     * @return le Graph pour lequel l'Edge est créée
     */
    Graph* getGraph();

    /**
     * obtient le Robot pour lequel l'Edge est créée
     * @return le Robot pour lequel l'Edge est créée
     */
    Robot* getRobot();

    /**
     * obtient la longueur de l'Edge
     * @return la longueur de l'Edge
     */
    double longueur();

    /**
     * obtient le Node initial de l'Edge
     * @return le Node initial de l'Edge
     */
    Node* getStart();

    /**
     * obtient le Node final de l'Edge
     * @return le Node final de l'Edge
     */
    Node* getEnd();

};

#endif


