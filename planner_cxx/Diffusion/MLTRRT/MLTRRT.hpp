// author: Romain Iehl <riehl@laas.fr>

#ifndef MLTRRT_HPP_INCLUDED
#define MLTRRT_HPP_INCLUDED

#include "planner_cxx/planner.hpp"
#include "planner_cxx/Diffusion/Expansion/RRTExpansion.h"
#include <tr1/memory>
#include <vector>
#include <set>
#include <utility>
#include "TransitionExpansion.h"
#include <boost/function.hpp>
#include "passive_zone.hpp"
#include "ml_bio_functions.hpp"

struct jnt;

double changeTemperature(double temp, double fromCost, double toCost, bool accept);

class compareNodes
{
public:
  bool operator()(std::pair<double, Node*> a, std::pair<double, Node*> b)
  {
    return(a.first <= b.first);
  }
};

class PathSegments
{
public:
  PathSegments(double totalLength, double segmentLength) :
    mNbSegments(ceil(std::max(0., (totalLength)) / std::max(10e-6, segmentLength))),
    mStep(std::max(0., (totalLength)) / mNbSegments)
  {
  }

  double operator[](unsigned i)
  {
    assert(i < mNbSegments + 1);
    return(i * mStep);
  }

  unsigned nbSegments()
  {
    return(mNbSegments);
  }

  unsigned nbPoints()
  {
    return(mNbSegments + 1);
  }

protected:
  unsigned mNbSegments;
  double mStep;
};

//------------------------------------------------------------------------------

class Nameless
{
public:
  Nameless(Node* _expansionNode,Node* _directionNode) :
    expansionNode(_expansionNode),
    directionNode(_directionNode),
    paths(),
    directionConfigReached(false),
    nbCreatedNodes(0)
  {}
  
  std::tr1::shared_ptr<Configuration> getExpansionConf()
  {
    return(paths.size() > 0 ?
	   paths.back()->getEnd() :
	   expansionNode->getConfiguration());
  }
  
  // in
  Node* expansionNode;
  Node* directionNode;
  // out
  std::vector<std::tr1::shared_ptr<LocalPath> > paths;
  bool directionConfigReached;
  unsigned nbCreatedNodes;
};

class ExtensionStepData
{
public:
  ExtensionStepData(std::tr1::shared_ptr<LocalPath> _directionPath) :
    directionPath(_directionPath),
    extensionPath(),
    interpolation(0.)
  {}

  // The extension is performed from directionPath.getBegin()
  // toward directionPath.getEnd()
  std::tr1::shared_ptr<LocalPath> directionPath;

  // Result of the extension : the path and the interpolation parameter.
  std::tr1::shared_ptr<LocalPath> extensionPath;
  double interpolation;
};

double step();

bool extend(ExtensionStepData& data, Env::expansionMethod method);

bool extendFixedStep(ExtensionStepData& data);

bool extendConnect(ExtensionStepData& data);

bool extendBoundedConnect(ExtensionStepData& data, double length);

std::vector<std::pair<double,double> > getPathMaxima(LocalPath& path);

// Function that balances the ratio of
// Exploration towards refinement

class MLTRRT : public Planner
{
  typedef std::tr1::shared_ptr<Configuration> ConfigurationPtr;

public:
  typedef struct jnt joint_t;

  MLTRRT(Robot* R, Graph* G);

  ~MLTRRT();

  virtual int init();

  virtual bool checkStopConditions();
  
  virtual bool preConditions();

  bool distanceStopCheck(Node* node);

  bool distanceStopCheck(ConfigurationPtr conf);
  
  virtual bool costConnectNodeToComp(Graph& G,
				     Node* N,
				     Node* compNode);

  /**
   * Expands tree from component fromComp,
   * to component toComp
   * @param fromComp the starting connex component
   * @param toComp the arriving connex component
   * @return the number of node created
   */
  virtual int expandOneStep(Node* fromComp, Node* toComp);
  
  void commitToGraph(Nameless& storage);

  void commitToGraphDirectConnection(Nameless& storage);

  void commitToGraphDirectConnection2(Nameless& storage);

  void commitToGraphCostConnection(Nameless& storage);

  /**
   * Main function of the Tree process
   * @return the number of Nodes added to the Graph
   */
  uint run();

  bool expandJointsByPerturbation(Nameless& storage, 
					  std::tr1::shared_ptr<Configuration> expansionConf,
				  std::vector<joint_t*> joints,
				  double step,
				  bool checkCost = false);

  bool expandPassiveIteratively(Nameless& storage,
				std::tr1::shared_ptr<Configuration> expansionConf,
				double step,
				std::vector<joint_t*> joints = std::vector<joint_t*>(),
				bool checkCost = false);
  
  void adjustTemperature(Node* node, double fromCost, double toCost, bool accept);

  /**
   * expansion des joints passifs dans le cas ML_RRT
   * @param expansionNode
   * @param NbActiveNodesCreated le nombre de Node créés lors de l'expansion de joints actifs
   * @return le nombre de Node Créés
   */
  bool expandProcess(Nameless& storage,
		     std::tr1::shared_ptr<Configuration> expansionConfig,
		     std::tr1::shared_ptr<Configuration> directionConfig); 
  /**
   * choisie si l'expansion sera de type Manhattan
   * @return l'expansion sera de type Manhattan
   */
  bool manhattanSamplePassive();

  bool isPathAcceptable(LocalPath& path, double temperature);

  bool isPathDescending(LocalPath& path);
  
  std::vector<Node*> getVisibleNodes(std::tr1::shared_ptr<Configuration> q, p3d_compco* comp);
  Node* getBestVisibleNode(std::tr1::shared_ptr<Configuration> q, p3d_compco* comp);

  std::vector<joint_t*> selectNewJoints(std::vector<joint_t*>& joints,
					std::vector<joint_t*>& oldJoints);
  
  int getNodeMethod() { return ExpansionNodeMethod; }
  
  /**
   * Expansion Step (Delta)
   */
  
  void addNodes(unsigned& nbCreatedNode, Node* fromNode, std::vector<std::tr1::shared_ptr<Configuration> >& configurations, Node* lastNode = NULL);

  /**
   * Function called when a node can not be connected
   * @param the node which has not been connected
   */
  void expansionFailed(Node& node);
  
  /**
   * Adds a node to a connected component
   */
  Node* addNode(Node* startNode, LocalPath& path, Node* endNode, unsigned& nbCreatedNodes);
 
    /**
     * Shoots a direction (includes the biasing)
     *
     * @param Expanding component
     * @param Goal Component
     * @param Sampling passive mode
     * @param Direction node
     */
    virtual std::tr1::shared_ptr<Configuration> getExpansionDirection(
            Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode);

    /**
     * Gets the nearest node in the graph
     *
     * @param compNode Expanding component
     * @param direction Direction of expansion
     * @param Sampling passive mode
     */
    virtual Node* getExpansionNode(
            Node* compNode, std::tr1::shared_ptr<Configuration> direction, int distance);

    /**
      * Basic Expansion Method
      *
      * @param expansionNode
      * @param directionConfig
      * @param method
      */
  bool expandProcessSimple(Nameless& storage,
			   std::tr1::shared_ptr<Configuration> expansionConfig,
			   std::tr1::shared_ptr<Configuration> directionConfig,
			   double step,
			   bool checkCost = false);  
    /**
     * Expands towards the goal
     *
     * @param expansionNode     Node expanded
     * @param directionConfig   Direction
     */
    bool expandToGoal(Node* expansionNode,
                      std::tr1::shared_ptr<Configuration> directionConfig);  
protected:
  boost::function<void(Configuration&, std::vector<joint_t*>&)> fPassiveShoot;
  boost::function<std::vector<joint_t*>(Configuration&)> mGetPassiveZoneJoints;
  boost::function<double(double, double, double, bool)> fChangeTemperature;
  boost::function<bool(Node*, ConfigurationPtr, double)> fCostTest;
  boost::function<std::vector<joint_t*>(Robot* R, Configuration& conf)> fGetCollidingJoints;
  boost::function<bool(Robot* R, Configuration&)> fGetCurrentInvalidConf;
  boost::function<void(Robot* R)> fResetCurrentInvalidConf;
  std::set<std::pair<double, Node*>, compareNodes> mSortedNodes;
  std::tr1::shared_ptr<LigandAtoms> mDistanceFilter;
  std::tr1::shared_ptr<LigandAtoms> mDistanceFilterStop;
  double mFilterMinDistance;
  int ExpansionNodeMethod;
  int MaxExpandNodeFailure;
  int kNearestPercent;
  
  int ExpansionDirectionMethod; // = GLOBAL_CS_EXP;
  //	double GoalBias; //= 0.1;
  //	bool IsGoalBias; //= FALSE;
  bool IsDirSampleWithRlg; //= FALSE;
  PassiveZone* mPassiveZone;
  TransitionExpansion mTransitionExpansion;
};

#endif
