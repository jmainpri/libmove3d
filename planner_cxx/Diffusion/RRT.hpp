#ifndef RRT_HPP
#define RRT_HPP

#include "Expansion/TreeExpansionMethod.hpp"
#include "../planner.hpp"
/**
 * @ingroup Diffusion
 *
 * ! \brief RRT
 *
 * This class implements the following RRT algorithms:<BR>
 * RRT, T-RRT and ML-RRT.<BR>
 * The expansion can be mono- or bi-directional,
 * with or without a goal.<BR>
 * The possible expansion methods are:<BR>
 * "extend", "extend n steps" and "connect".<BR>
 * There are some restrictions on the use of those methods:<BR>
 * connect cannot be used with T-RRT,
 * while ML-RRT should be used with connect.
 */
class RRT : public Planner
{

public:
    /** Constructor from a WorkSpace object
     * @param WS the WorkSpace
     */
    RRT(WorkSpace* WS);

    /**
     * Destructor
     */
    ~RRT();

    /**
     * The number of consecutive failure
     * @return a signed integer of the number of consecutuive failure
     */
    int getNbFailures();

    /**
     * Initialzation of the plannificator
     * @return the number of node added during the init phase
     */
    int init();

    /**
     * Checks out the Stop condition
     * @param (*fct_stop)(void) the stop function
     * @return true if the plannification must stop
     */
    bool checkStopConditions();

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
     * Function called when a node can not be connected
     * @param the node which has not been connected
     */
    void expansionFailed(Node* node);

    /**
     * Connects a nez node to the Graph
     * @param currentNode The node to which the new node will be connected
     * @param path between the new node and the nearest node
     * @param pathDelta in/out the delta along the path
     * @param directionNode the extention direction
     * @param currentCost the cost of current node
     * @param nbCreatedNodes in/out Number of nodes created
     * @return the new node
     */
    Node* connectNode(Node* currentNode, LocalPath& path, double pathDelta,
    		Node* directionNode, double currentCost, int& nbCreatedNodes);

    /**
     * sets up the new temperature
     * @param node The node
     */
    void adjustTemperature(bool accepted, Node* node);

    /**
     * Gets the maximal step for an extention
     * @return The maximal step
     */
    double step() {return(p3d_get_env_dmax() * ENV.getDouble(Env::extensionStep));}

    /**
     * Shoots a new configuration randomly at a fix step
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
     * Main function of the RRT process
     * @return le nombre de Node ajoutés au Graph
     */
    uint run();

private:
    TreeExpansionMethod* Expansion;
    int _nbConscutiveFailures;

};

#endif
