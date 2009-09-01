#ifndef P3D_DIFFUSION_PROTO_HH
#define P3D_DIFFUSION_PROTO_HH

#include "../planner/PlanningAPI/planningAPI.hpp"
#include "Expansion/TreeExpansionMethod.hpp"
#include "../../qtWindow/qtBase/env.hpp"

#include "traj.h"
#include "roadmap.h"

#include <tr1/memory>


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
class RRT {

public:
	RRT(p3d_graph* G, int (*StopFunction)(), void (*DrawFunction)());

	~RRT();

	uint run();
	void reset();
	bool trajFound();
	void writeTrajectoryPdbs(std::string filename, double granularity, Dist* dist);

protected:

	std::tr1::shared_ptr<Configuration> diffuseOneConf(
			std::tr1::shared_ptr<Configuration> qCurrent);

	void connectNodeToComp(Node* node, Node* compNode);
	bool manhattanSamplePassive();
	int expandOneStep(Node* fromComp, Node* toComp);
	int expandOneStepManhatan(Node* fromComp, Node* toComp);

	int expandProcess(Node& expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig,
			Node* directionNode,
			Env::expansionMethod method);

	void addCycles(Node& node);

	Node* insertNode(std::tr1::shared_ptr<Configuration> q,
			Node* expansionNode,
			double expansionDist,
			double currentCost);

	Node* connectNode(Node* currentNode,
			Localpath& path,
			double pathDelta,
			Node* directionNode,
			double currentCost,
			int& nbCreatedNodes);

	void checkStopByWeight(Node* n);

	bool checkStopConditions();

	bool costTestSucceeded(Node* previousNode,
			Configuration& currentConfig,
			double currentCost);

	void adjustTemperature(bool accepted, Node* node);

	void costConnectNodeToComp(Node* node,
			Node* compNode);

	bool costTestSucceededConf(
			std::tr1::shared_ptr<Configuration>& previousConfig,
			std::tr1::shared_ptr<Configuration>& currentConfig,
			double temperature);

	bool ExpandToGoal(Node* expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig );

	bool ExpandCostConnect(Node& expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig,
			Node* directionNode,
			Env::expansionMethod method,
			bool toGoal);

	int passiveExpandProcess(Node* expansionNode, int NbActiveNodesCreated, Node* directionNode);

	// Fonction de test d'expansion vers le But
	double adjustTemperature(bool accepted,double temperature);

private:
	TreeExpansionMethod* Expansion;

//TODO Members private
protected:
	int mConsecutiveFailures;

	int (*_stop_func)();
	void (*_draw_func)();

	Robot* mR;

	Graph* mG;

	Node* mStart;
	Node* mGoal;
};

extern RRT* rrt;
extern FILE* traj_file;

/*! \brief Main function to run a diffusion process.
 *
 * @param GraphPt the graph to expand, if set to NULL a new graph will be created.
 * @param fct_stop function to refresh the UI and check if the user has requested a halt of the diffusion process.
 * @param fct_draw function to redraw the scene
 * @return true if the start and goal configurations are linked.
 */
bool p3d_RunDiffusion(p3d_graph* GraphPt,
		int (*fct_stop)(void),
		void (*fct_draw)(void));
#endif


