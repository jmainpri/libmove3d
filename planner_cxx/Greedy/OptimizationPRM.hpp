#ifndef P3D_GREEDY_HYBRID_PROTO_HH
#define P3D_GREEDY_HYBRID_PROTO_HH

#include <map>
#include <vector>
#include <iostream>
#include <tr1/memory>

#include "../PlanningAPI/planningAPI.hpp"
#include "../Diffusion_cxx/RRT.hpp"
#include "../../qtWindow/qtBase/env.hpp"
#include "../planner/PlanningAPI/Trajectory/CostOptimization.hpp"

class GreedyHybrid {

public:

	GreedyHybrid(p3d_graph* G, int (*stop_func)(), void (*draw_func)());
	~GreedyHybrid();

	int run();

	bool getTrajExist(){ return traj_exist;}
	void createVectorLocalPath();

	int strait(Node& expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig,
		     Node* directionNode,
		     Env::expansionMethod method,
		     bool toGoal);

	void optimizePhaze();
	void optimizeLinear();
	void shortCutLinear();
	bool checkStopConditions();

private:



	bool traj_exist;
	int mConsecutiveFailures;

	int (*_stop_func)();
	void (*_draw_func)();

	Robot* mRobot;
	Graph* mGraph;

	Node* mStart;
	Node* mGoal;

	TreeExpansionMethod* Expansion;
	RRT* Diffusion;
	CostOptimization* optimTrj;

	int nb_Loops;
	int nb_LocalPaths;
	int nb_CostCompare;

	int nb_CObstFail;

};

extern GreedyHybrid* OptPlanner;

extern bool p3d_RunGreedyHybrid(p3d_graph* GraphPt, int (*fct_stop)(void),
		void (*fct_draw)(void));

#endif
