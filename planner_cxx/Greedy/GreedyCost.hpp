#ifndef P3D_GREEDY_PROTO_HH
#define P3D_GREEDY_PROTO_HH

#include "../../planner_cxx/API/planningAPI.hpp"
#include "../../planner_cxx/API/Trajectory/CostOptimization.hpp"
#include "../../planner_cxx/API/Trajectory/BaseOptimization.hpp"

#include "../Diffusion/RRT.hpp"
#include "../Diffusion/RRT-Variants/Transition-RRT.hpp"

class GreedyCost {

public:

	GreedyCost(p3d_graph* G, int (*stop_func)(), void (*draw_func)());
	~GreedyCost();

	bool run();

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

	int getOptimFail() { return nb_OptimFail; }
	int getOptimSuccess() {return nb_OptimSuccess; }

private:

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

	int nb_OptimSuccess;
	int nb_OptimFail;

};

extern bool p3d_RunGreedyCost(p3d_graph* GraphPt, int (*fct_stop)(void),
		void (*fct_draw)(void));

#endif
