#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "../planningAPI.hpp"

/**
	\brief Classe représentant un Graph pour un Robot
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Graph{

public:
  //contructors and destructor
    /**
     * Constructeur de la classe
     * @param G la structure p3d_graph qui sera stockée
     * @param R le Robot pour lequel le Graph est créé
     */
    Graph(Robot* R, p3d_graph* ptGraph);
    /**
     * Constructeur de classe
     * @param R le Robot pour lequel le Graph est créé
     */
    Graph(Robot* R);
    /**
     * Constructeur de la classe
     * !!Attention!! ce constructeur n'est à utiliser que si le Robot pour lequel le Graph est créé n'existe pas
     * @param G la structure p3d_graph qui sera stockée
     */
    Graph(p3d_graph* G);

    /**
     * Destructeur de la classe
     */
    ~Graph();


  //Accessors
    /**
     * obtient la structure p3d_graph stockée
     * @return la structure p3d_graph stockée
     */
    p3d_graph* getGraphStruct();
    /**
     * stocke la structure p3d_graph
     * @param G la structure à stocker
     */
    void setGraph(p3d_graph* G);

    /**
     * obtient le Robot pour lequel le Graph est créé
     * @return le Robot pour lequel le Graph est créé
     */
    Robot* getRobot();

    /**
     * modifie la trajectoire stockée
     * @param Traj la nouvelle trajectoire à stocker
     */
    void setTraj(p3d_traj* Traj);
    /**
     * obtient la trajectoire stockée
     * @return la trajectoire stockée
     */
    p3d_traj* getTrajStruct();

    /**
     * obtient le vecteur des Node du Graph
     * @return le vecteut des Node du Graph
     */
    std::vector<Node*> getNodes();
    /**
     * obtient le vecteur des Edge du Graph
     * @return le vecteur des Edge du Graph
     */
  std::vector<Edge*> getEdges();

    /**
     * obtient la table des noeuds du Graph
     * @return la table des noeuds du Graph
     */
  std::map<p3d_node*, Node*> getNodesTable();

    /**
     * obtient le nombre de Node dans le Graph
     * @return le nombre de Node dans le Graph
     */
    int getNbNode();
    /**
     * obtient le Node correspondant au p3d_node
     * @param N le p3d_node
     * @return le Node correspondant; NULL si le Node n'existe pas
     */
    Node* getNode(p3d_node* N);

    /**
     * obtient le dernier Node ajouté au Graph
     * @return le dernier Node ajouté au Graph
     */
    Node* getLastnode();

    /**
     * obtient le nom du Graph
     * @return le nom du Graph
     */
    std::string getName();
    /**
     * modifie le nom du Graph; un nom standard est alors mis
     */
    void setName();
    /**
     * modifie le nom du Graph
     * @param Name le nouveau nom
     */
    void setName(std::string Name);


    /**
      * Gets Start Node
      * @return le nom du Graph
      */
    Node* getStart() {return _Start;}

    /**
      * Sets Start Node
      */
    void setStart(Node* N) { _Start=N; }


    /**
      * Gets Start Node
      * @return le nom du Graph
      */
    Node* getGoal() {return _Goal;}

    /**
      * Sets Goal Node
      */
    void setGoal(Node* N) { _Goal=N; }


    /**
     * teste si deux Graph sont égaux en comparant leur listNode
     * @param G le Graph à comparer
     * @return les deux Graph sont égaux
     */
    bool equal(Graph* G);
    /**
     * teste si deux Graph ont le même nom
     * @param G le Graph à comparer
     * @return les deux Graph ont le même nom
     */
    bool equalName(Graph* G);

    /**
     * obtient le Node correspondant à la Configuration
     * @param q la Configuration recherchée
     * @return le Node s'il existe, NULL sinon
     */
    Node* searchConf(std::tr1::shared_ptr<Configuration> q);
    /**
     * insert un Node dans le Graph
     * @param node le Node à insérer
     * @return le Node a été inséré
     */
    Node* insertNode(Node* node);
    /**
     * insert un Node de type ISOLATE
     * @param node le Node à insérer
     * @return le Node a été inséré
     */
    Node* insertExtremalNode(Node* node);

    /**
     * trie les Edges en fonction de leur longueur
     */
    void sortEdges();
    /**
     * ajoute un Edge au Graph
     * @param N1 le Node initial de l'Edge
     * @param N2 le Node final de l'Edge
     * @param Long la longueur de l'Edge
     */
    void addEdge(Node* N1, Node* N2, double Long);
    /**
     * ajoute deux Edge au Graph
     * @param N1 l'une des extrémités des Edge
     * @param N2 l'autre extrémité des Edge
     * @param Long la longueur des Edge
     */
    void addEdges(Node* N1, Node* N2, double Long);

    /**
     * trie les Node en fonction de leur distance à un Node de référence
     * @param N le Node de référence
     */
    void sortNodesByDist(Node* N);
    /**
     * ajoute un Node s'il est à moins d'une distance max du Graph
     * @param N le Node à ajouter
     * @param maxDist la distance max
     */
    void addNode(Node* N, double maxDist);
    /**
     * ajoute des Node s'ils sont à moins d'une distance max du Graph
     * @param N les Node à ajouter
     * @param maxDist la distance max
     */
  void addNodes(std::vector<Node*> N, double maxDist);

    /**
     * vérifie si un node est dans le Graph
     * @param N le Node à tester
     * @return le Node est dans le Graph
     */
    bool isInGraph(Node* N);
    /**
     * lie un Node au Graph
     * @param N le Node à lier
     * @return le Node est lié
     */
    bool linkNode(Node* N);
    /**
     * Lie un node au Graph sans prendre en compte la distance max
     * @param N le Node à lier
     * @return le Node est lié
     */
    bool linkNodeWithoutDist(Node* N);
    /**
     * Lie un Node au Graph en prenant en compte la distance max
     * @param N le Node à lier
     * @return le Node est lié
     */
    bool linkNodeAtDist(Node* N);

    /**
     * lie un Node à tous les autres Nodes visibles
     * @param N le Node à lier
     * @return le Node est lié
     */
    bool linkToAllNodes(Node* N);

    /**
     * test si un node est linkable en suivant la visibilité
     * @param N le Node à lier
     * @param link in/out le nombre de composantes connexes auxquelles le node peut être lié
     * @return le vecteur des composantes connexes auxquelles le Node peut être lié
     */
  std::vector<Node**> isOrphanLinking(Node* N, int* link);

    /**
     * lie un Node en suivant la visibilité
     * @param N le Node à lier
     * @param Graph_Pt le Graph affiché
     * @param (*fct_draw)(void) la fonction d'arret
     * @param type le type de Node que l'on veut ajouté (gradien:0 ou connecteur:1 ou indifférent:2)
     * @param ADDED in/out le nombre de Node ajoutés
     * @param nb_fail in/out le nombre d'échecs consecutifs
     * @return le Node est lié
     */
    bool linkOrphanLinking(Node* N, p3d_graph* Graph_Pt, void (*fct_draw)(void), int type, int* ADDED, int* nb_fail);

     /**
      * crée un Node dans le graph en suivant la visibilité
      * @param Graph_Pt le Graph affiché
      * @param (*fct_draw)(void) la fonction d'arret
      * @param type le type de Node que l'on veut créé (gradien:0 ou connecteur:1 ou indifférent:2)
      * @param ADDED in/out le nombre de Node créés
      * @param nb_fail in/out le nombre d'échecs consecutifs
      */
     void createOneOrphanLinking(p3d_graph* Graph_Pt, void (*fct_draw)(void), int type, int* ADDED, int* nb_fail);

    /**
     * crée des Node dans le Graph en suivant la visibilité
     * @param nb_node le nombre de Node à créer
     * @param (*fct_stop)(void) la fonction d'arret
     * @param (*fct_draw)(void) la fonction d'affichage
     * @param type le type de Node que l'on veut créé (gradien:0 ou connecteur:1 ou indifférent:2)
     * @return le nombre de Node créés
     */
    int createOrphansLinking(int nb_node, int (*fct_stop)(void), void (*fct_draw)(void), int type);

    /**
     * crée des Node à des Configurations aléatoires
     * @param NMAX le nombre de Node à crér
     * @param (*fct_stop)(void) la fonction d'arret
     * @param (*fct_draw)(void) la fonction d'affichage
     */
    void createRandConfs(int NMAX, int (*fct_stop)(void), void (*fct_draw)(void));

    Node* randomNodeFromComp(Node* comp);

    /**
     * obtient le plus proche voisin d'une composante connexe
     * @param compco la composante connexe dans laquelle on cherche le Node
     * @param C la Configuration dont on veut determier le plus proche voisin
     * @param weighted
     * @param distConfigChoice le type de calcul de distance
     * @return le Node le plus proche de la Configuration appartenant à la composante connexe
     */
  Node* nearestWeightNeighbour(Node* compco, std::tr1::shared_ptr<Configuration> C, bool weighted, int distConfigChoice);

    /**
     * ajoute des Edges formant des cycles dans le Graph
     * @param node le nouveau Node ajouté
     * @param step la distance d'expansion
     */
    void addCycles(Node* node, double step);

    /*fonction mergeant les compco N1 et N2*/
    /**
     * merge deux composantes connexes
     * @param CompCo1 la première composante connexe
     * @param CompCo2 la seconde composante connexe
     * @param DistNodes la distance entre les deux composantes connexes
     * @return les deux composantes sont mergées
     */
    int MergeComp(Node* CompCo1, Node* CompCo2, double DistNodes);
    /**
     * teste si des composantes connexes doivent être merger et le fait le cas échéant
     */
    void MergeCheck();

    /**
     * \brief Link a Configuration to a Node for the RRT Planner
     * @param q the Configuration to link
     * @param expansionNode the Node
     * @param currentCost the current cost to reach the Node
     * @param step the max distance for one step of RRT expansion
     * @return the inserted Node
     */
  Node* insertNode(Node* expansionNode, LocalPath& path);

    /**
     * Link a Configuration to a Node for the RRT Planner
     * @param q the Configuration to link
     * @param from the Node
     * @return the linked Node
     */
  Node* insertRrtLinkingNode(std::tr1::shared_ptr<Configuration> q, Node* from, double step );

  /**
   * Number of Nodes
   */
  unsigned int getNumberOfNodes() { return _Nodes.size(); }

private:

  void init();
  void freeResources();
  static bool compareEdges(Edge* E1, Edge* E2);
  static bool compareNodes(Node* N1, Node* N2);
  bool isEdgeInGraph(Node* N1,Node* N2);

private:
  p3d_graph* _Graph;
  Robot* _Robot;

  p3d_traj* _Traj;

  std::vector<Node*> _Nodes;
  std::vector<Edge*> _Edges;
  std::map<p3d_node*, Node*> _NodesTable;

  Node* _Start;
  Node* _Goal;

  std::string _Name;


};

#endif
