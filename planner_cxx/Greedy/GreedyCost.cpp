
#include "GreedyCost.hpp"

using namespace std;
using namespace tr1;



GreedyCost::GreedyCost(p3d_graph* G,
		int (*stop_func)(),
		void (*draw_func)()) :

			mConsecutiveFailures(0),
			_stop_func(stop_func),
			_draw_func(draw_func),

			mRobot(new Robot(G->rob)),
			mGraph(new Graph(G ? G : p3d_create_graph(),mRobot)),
					mStart(NULL),
					mGoal(NULL) {

	if (!ENV.getBool(Env::isCostSpace)){
		cout << "Error Not Cost Space" << endl;
	}

//	Configuration goalConf(*mRobot->getGoTo());
//	Configuration startConf(*mRobot->getInitialPosition());

	Expansion = new TreeExpansionMethod(mGraph);

	nb_Loops=0;
	nb_LocalPaths=0;
	nb_CostCompare=0;
	nb_CObstFail=0;

	nb_OptimSuccess=0;
	nb_OptimFail=0;
}

GreedyCost::~GreedyCost(){
/*	delete Expansion;
	delete trajPt;*/
}


bool GreedyCost::run() {

	cout << "-------------- Start Diffusion ----------------------" << endl;

	WorkSpace* ws = new WorkSpace("MainEnv");

	Diffusion = new RRT(ws);

	ENV.setBool(Env::isCostSpace,false);
	ENV.setExpansionMethod(Env::Connect);
	ENV.setBool(Env::biDir,true);
	Diffusion->init();
	mGraph = Diffusion->getActivGraph();
	int nb_nodes = Diffusion->run();
	ENV.setBool(Env::isCostSpace,true);

	if( Diffusion->trajFound() ){

		mGraph->setTraj(mRobot->getTrajStruct());
		cout << "Trajectory exists" << endl;
		ENV.setBool(Env::drawTraj,true);
		g3d_draw_allwin_active();

		cout << "-------------- Start Optimization ----------------------" << endl;

		p3d_ExtractBestTraj(mGraph->getGraphStruct());

		optimTrj = new CostOptimization(mRobot,mRobot->getTrajStruct());

//		cout << "--- Remove redundant nodes ---" << endl;
//
//		optimTrj->removeRedundantNodes();
//		optimTrj->replaceP3dTraj(mGraph->getP3DTraj());
//
//		g3d_draw_allwin_active();

		optimizeLinear();

		cout << "--- Remove redundant nodes ---" << endl;

//		optimTrj->cutTrajInSmallLP();

//		optimTrj->removeRedundantNodes();

//		trajToDraw.resize(1);
//		trajToDraw.at(0) = static_cast<Trajectory>(*optimTrj);

		g3d_draw_allwin_active();

		p3d_PrintTrajCost(
				mGraph->getGraphStruct(),
				mRobot->getTrajStruct());

		optimTrj->replaceP3dTraj();

		double dmax=0;
		p3d_col_get_dmax(&dmax);

		cout << " DMax = " << dmax << endl;

		cout << "-------------- End Greedy ------------------------" << endl;

		cout << "Global min of search = " << optimTrj->getMinCost() << endl;
		cout << "Actual  = " << optimTrj->cost() << endl;

		if(ENV.getBool(Env::debugCostOptim)){
			optimTrj->printDebugInfo();
		}

		delete optimTrj;
	}

	return Diffusion->trajFound();

}

void GreedyCost::shortCutLinear() {

	int nb_success(0);
	int nTotFail(0);
	int nLoopTotShort(0);
	int nFailOptim(0);

	const int short_cut_max = 300;

	const int maxFactorShort= 3;
	const int minFactorShort= 1;

	double factor = (double)maxFactorShort;
	bool isOptimSuccess(false);

	while(nLoopTotShort<short_cut_max){

		factor = factor - (maxFactorShort-minFactorShort)/ (double)short_cut_max;

		nLoopTotShort++;

		// TODO
		//isOptimSuccess = optimTrj->oneLoopShortCut(factor);

		if(isOptimSuccess == false){
			nFailOptim++;
			nb_OptimFail++;
		} else {

			optimTrj->replaceP3dTraj();

			g3d_draw_allwin_active();
			nFailOptim = 0;
			nb_OptimSuccess++;
		}

	}

	cout << "------------- Short Cut --------------------" << endl;
	cout << "Factor = " << factor << endl;
	cout << "Nb Success = " << nb_success << endl;
	cout << "Nb Fail = " << nTotFail << endl;
	cout << "Nb Loops = " << nLoopTotShort << endl;

}
void GreedyCost::optimizeLinear() {

	//	const double FactorMax(1000);
	//	const double FactorMin(0.5);

	double dmax=0;
	p3d_col_get_dmax(&dmax);
	dmax = p3d_get_env_dmax();
	cout << "dmax = " << dmax << endl;
	cout << "RangeMax = " << optimTrj->getRangeMax() << endl;
//
//	ENV.setDouble()

	double maxFactor = dmax*(ENV.getDouble(Env::MaxFactor));
	double minFactor = dmax;

	double factor(0.0);

	int nb_success(0);
	int nTotFail(0);
	int nFailOptim(0);
	int nLoopTot(0);
	int isOptimSuccess(0);
	//	int MaxCostFail = 10 * ENV.getInt(Env::maxCostOptimFailures);
	const int nLoopTotMax(10000);
	const int nMaxCostFail(1000);

	factor = maxFactor;

	while(/*(nFailOptim < nMaxCostFail)*/ true
			&& (nLoopTot < nLoopTotMax)
			&& (nLoopTot < ENV.getInt(Env::nbCostOptimize))
			&& !checkStopConditions() ) {

		factor = factor - ((maxFactor-minFactor)/(double)ENV.getInt(Env::nbCostOptimize));

		nLoopTot++;


//		cout << ((double)ENV.getInt(Env::nbCostOptimize))*4/5 << endl;

//		if( /*((double)nLoopTot)>(((double)ENV.getInt(Env::nbCostOptimize))*4/5)&&*/ (nLoopTot%10)==1 ){
//			isOptimSuccess = optimTrj->oneLoopShortCut();
//		}
//		else{
			isOptimSuccess = optimTrj->oneLoopDeform(/*p3d_random(minFactor,factor)*/factor);
//			cout << "Factor = " << factor << endl;
//		}

		if(isOptimSuccess == false) {
			nb_OptimFail++;
			nTotFail++;
		} else {
//			cout << mGraph->getP3DTraj() << endl;
			optimTrj->replaceP3dTraj();
			g3d_draw_allwin_active();
			nFailOptim = 0;
			nb_OptimSuccess++;

		}
	}

	cout << "------------------ Optimize Linear ---------------------" << endl;
	cout << "Factor = " << factor << endl;
	cout << "Nb Success = " << nb_success << endl;
	cout << "Nb Fail = " << nTotFail << endl;
	cout << "Nb Loops = " << nLoopTot << endl;
	cout << "--------------------------------------------------------" << endl;

	nb_success = 0;
	nTotFail = 0;
	nFailOptim = 0;
	nLoopTot = 0;
	ENV.setBool(Env::drawTraj,true);
}

void GreedyCost::optimizePhaze(){

	int nFailOptim=0;
	int nLoopTot=0;
	int nLoopTotMax=2000;
	int isOptimSuccess;
	//mGraph->

	const int IterationMax(3);
	//	const double FactorMax(1000);
	//	const double FactorMin(0.5);

	std::vector<int> MaxCostFail;


	for(int i=0;i<IterationMax;i++){
		MaxCostFail.push_back(ENV.getInt(Env::maxCostOptimFailures)/(i+1));
	}

	std::vector<int> Factor;

	Factor.push_back(100);
	Factor.push_back(10);
	Factor.push_back(1);

	double factor(0.0);
	double it(0.0);

	int i = 0;
	int nb_success(0);
	int nTotFail(0);

	while(i < IterationMax){
		//Loop done until an optimization failed a given number of times or when it reaches
		// a maximal number of loops
		//		it = (double)i;
		//		factor = ((FactorMin-FactorMax)/(double)IterationMax)*(it+1)+FactorMax;
		factor = Factor.at(i);

		while((nFailOptim < MaxCostFail.at(i) )
				&& (nLoopTot < nLoopTotMax) ) {
			nLoopTot++;
			isOptimSuccess = optimTrj->oneLoopDeform(factor);

			if(isOptimSuccess == false)
			{
				nFailOptim ++;
				nTotFail++;
			}
			else
			{
				nFailOptim = 0;
				nb_success++;
				g3d_draw_allwin_active();
			}
		}
		i++;
		cout << "Factor = " << factor << endl;
		cout << "Nb Success = " << nb_success << endl;
		cout << "Nb Fail = " << nTotFail << endl;
		cout << "Nb Loops = " << nLoopTot << endl;
		cout << "--------------------------------------------"<< endl;

		nb_success=0;
		nTotFail=0;
		nFailOptim = 0;
		nLoopTot=0;
		ENV.setBool(Env::drawTraj,true);
	}

	cout << "nb_Loops = " << nb_Loops << endl;
	cout << "nb_LocalPaths = " << nb_LocalPaths << endl;
	cout << "nb_CostCompare = " << nb_CostCompare << endl;
	cout << "nb_CObstFail = " << nb_CObstFail << endl;
}

bool GreedyCost::checkStopConditions()
{

	if (!(*_stop_func)())
		p3d_SetStopValue(true);
	if(p3d_GetStopValue())
	{
		cout << "Greedy search canceled." << endl;
		return(true);
	}
	return(false);
}

bool p3d_RunGreedyCost(p3d_graph* GraphPt, int (*fct_stop)(void),
		void (*fct_draw)(void)) {

	cout << endl
	<< "**************************************" << endl
	<< " Beginning of Greedy search process" << endl << endl;

	MY_ALLOC_INFO("Before the graph creation");
	double tu,ts;

	if(!GraphPt){
		//p3d_del_graph(GraphPt);
		GraphPt = p3d_create_graph();
		}

	p3d_rob* RobotPt = GraphPt->rob;

	int nbAddedNodes = 0;
	p3d_jnt_set_dof_is_active_for_planner(RobotPt->joints[0], 3, false);
	p3d_jnt_set_dof_is_active_for_planner(RobotPt->joints[0], 4, false);
	p3d_jnt_set_dof_is_active_for_planner(RobotPt->joints[0], 5, false);

	ChronoOn();

	GreedyCost* OptPlanner = new GreedyCost(GraphPt,fct_stop,fct_draw);

	bool trajExists = OptPlanner->run();

	ChronoPrint("");
	ChronoTimes(&tu,&ts);
	GraphPt->time = GraphPt->time + tu;

	ChronoOff();

	cout << "Nb. of fail : " << OptPlanner->getOptimFail() << endl;
	cout << "Nb. of success : " << OptPlanner->getOptimSuccess() << endl;

	cout << endl
	<< " End of Greedy search process" << endl
	<< "**************************************" << endl << endl;


	if(trajExists==false)
	{
		trajExists = false;
		cout << "No solution path: the exploration didn't \
		link a start and a goal configuration." << endl;
	}

	g3d_draw_allwin_active();

	cout << endl;

	delete OptPlanner;
	return trajExists;
}