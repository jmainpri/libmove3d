#ifndef RRT_HPP
#define RRT_HPP

#include "../planner.hpp"
/**
	\brief Classe représentant l'algorithme RRT
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class RRT : public Planner
{
private:
    int _nbConscutiveFailures;

public:
    /*construit un RRT a partir d'un WorkSpace*/
    /**
     * Constructeur de la classe
     * @param WS Le WorkSpace de l'application
     */
    RRT(WorkSpace* WS);

    /**
     * Destructeur de la classe
     */
    ~RRT();

    /**
     * obtient le nombre d'śchecs consécutifs pendant la planification
     * @return le nombre d'échecs consécutifs pendant la planification
     */
    int getNbFailures();

    /**
     * initialise le Planner
     * @return le nombre de Node ajoutés lors de l'initialisation
     */
    int init();

    /**
     * test les conditions d'arret
     * @param (*fct_stop)(void) la fonction d'arret
     * @return l'algorithme doit s'arreter
     */
    bool checkStopConditions(int (*fct_stop)(void));

    /**
     * génére un nouveau LocalPath
     * @param path un LocalPath
     * @param directionNode une direction
     * @param pathDelta in/out le pathDelta
     * @param newPath in/out le nouveau LocalPath
     * @param method le type de méthode d'extention
     * @return le nouveau LocalPath est valide
     */
    bool nextStep(LocalPath& path, Node* directionNode, double& pathDelta,
    		std::tr1::shared_ptr<LocalPath>& newPath, Env::expansionMethod method);

    /**
     * fonction appellée lors de l'échec de connection d'un Node
     * @param node le Node qui n'a pas été connecté
     */
    void expansionFailed(Node* node);

    /**
     * connect un nouveau Node au Graph
     * @param currentNode le Node auquel le nouveau Node sera connecté
     * @param path in/out le LocalPath entre le currentNode est le directionNode
     * @param pathDelta in/out le pathDelta
     * @param directionNode la direction d'extention
     * @param currentCost le cout pour atteindre le Node currentNode
     * @param nbCreatedNodes in/out le nombre de Node créés
     * @return le nouveau Node
     */
    Node* connectNode(Node* currentNode, LocalPath& path, double pathDelta,
    		Node* directionNode, double currentCost, int& nbCreatedNodes);

    /**
     * ajuste la temperature du Node
     * @param node le Node
     */
    void adjustTemperature(bool accepted, Node* node);

    /**
     * obtient le pas maximum de la diffusion d'un Node
     * @return le pas maximum de la diffusion d'un Node
     */
    double step() {return(p3d_get_env_dmax() * ENV.getDouble(Env::extensionStep));}

    /**
     * tire une Configuration dans une direction aléatoire à une distance finie d'une Configuration donnée
     * @param qCurrent la Configuration limitant la distance
     * @return la Configuration tirée
     */
    std::tr1::shared_ptr<Configuration> diffuseOneConf(std::tr1::shared_ptr<Configuration> qCurrent);

    /**
     * expansion de Node de la composant connexe fromCompco vers toCompco
     * @param fromComp la composante connexe de départ
     * @param toComp la composante connexe d'arrivée
     * @return le nombre de Node créés
     */
    int expandOneStep(Node* fromComp, Node* toComp);

    /**
     * 
     * @param expansionNode 
     * @param directionConfig 
     * @param directionNode 
     * @param method 
     * @return 
     */
    int ExpandProcess(Node* expansionNode, std::tr1::shared_ptr<Configuration> directionConfig,
    		Node* directionNode, Env::expansionMethod method);

    /**
     * expansion des joints passifs dans le cas ML_RRT
     * @param expansionNode 
     * @param NbActiveNodesCreated le nombre de Node créés lors de l'expansion de joints actifs
     * @param directionNode la direction de l'expansion
     * @return le nombre de Node Créés
     */
    int passiveExpandProcess(Node* expansionNode, int NbActiveNodesCreated, Node* directionNode);

    /**
     * 
     * @param path 
     * @param positionAlongDirection 
     * @param compNode 
     * @return 
     */
    bool expandControl(LocalPath& path, double positionAlongDirection, Node* compNode);

    /**
     * choisie si l'expansion sera de type Manhattan
     * @return l'expansion sera de type Manhattan
     */
    bool manhattanSamplePassive();

    int selectNewJntInList(p3d_rob *robotPt, std::vector<p3d_jnt*>& joints,
    			   std::vector<p3d_jnt*>& oldJoints, std::vector<p3d_jnt*>& newJoints);

    int getCollidingPassiveJntList(p3d_rob *robotPt, configPt qinv,
    			       std::vector<p3d_jnt*>& joints);

    void shoot_jnt_list_and_copy_into_conf(p3d_rob *robotPt, configPt qrand,
		       std::vector<p3d_jnt*>& joints);

    /**
     * fonction principale de l'algorithme RRT
     * @param Graph_Pt le graphPt affiché
     * @param (*fct_stop)(void) la fonction d'arret
     * @param (*fct_draw)(void) la fonction d'affichage
     * @return le nombre de Node ajoutés au Graph
     */
    uint expand(p3d_graph* Graph_Pt,int (*fct_stop)(void), void (*fct_draw)(void));

};

#endif
