/*
 * CostOptimization.cpp
 *
 *  Created on: Jun 25, 2009
 *      Author: jmainpri
 */

#include "CostOptimization.hpp"


using namespace std;
using namespace tr1;

CostOptimization::CostOptimization() :
	mincost(numeric_limits<double>::max()),
	nbErrors(0),
	DeformBiased(true){
	Errors.resize(0);
}

CostOptimization::CostOptimization(const Trajectory& T) :
	BaseOptimization(T),
	mincost(numeric_limits<double>::max()),
	nbErrors(0),
	DeformBiased(true){

	Errors.resize(0);

}

CostOptimization::CostOptimization(Robot* R, p3d_traj* t) :
	BaseOptimization(R,t),
	mincost(numeric_limits<double>::max()),
	nbErrors(0),
	DeformBiased(true){

	Errors.resize(0);


}

CostOptimization::~CostOptimization(){

	//	cout << "Delete CostOptimization" << endl;

}

bool CostOptimization::oneLoopDeform(double step) {

	double lPrev =0., lCurrent=0., lNext=0.;

	step *= p3d_get_env_dmax();

	bool isOptimSuccess(false);

	//Get 3 configurations at random along the trajectory
	//p3d_GetRandSuccesConfAlongTraj(trajPt,
	vector< shared_ptr<Configuration> > vectConf;
	//
	//	vectConf = get3RandSuccesConfAlongTraj(lPrev,lCurrent,lNext,step);

	shared_ptr<Configuration> qRandPtr = getRobot()->shoot();

	vectConf = getClosestConfOnTraj(lPrev,lCurrent,lNext,qRandPtr,step);

	shared_ptr<Configuration> qPrevPt = 	vectConf.at(0);
	shared_ptr<Configuration> qCurrentPt = 	vectConf.at(1);
	shared_ptr<Configuration> qNextPt = 	vectConf.at(2);



	// Take a configuration in a certain direction
	//	LocalPath path(qCurrentPt,getRobot()->shoot());
	LocalPath path(qCurrentPt,qRandPtr);

	shared_ptr<Configuration> qNewPt = path.configAtParam(step/2);
	vectConf.at(1) = qNewPt;

	// If qNew is free then
	// Check the triangle localPath
	if(!qNewPt->IsInCollision()){

		LocalPath* FirstHalf = new LocalPath(qPrevPt,qNewPt);
		LocalPath* SecondHalf = new LocalPath(qNewPt,qNextPt);

		// If the path is valid
		// Check for cost
		if( FirstHalf->getValid() && SecondHalf->getValid()){

			double sumOfCost = FirstHalf->cost() + SecondHalf->cost();
			double costOfPortion = this->costOfPortion(lPrev,lNext);

			if( sumOfCost <  costOfPortion  ){

				vector<LocalPath*> paths;

				paths.push_back(FirstHalf);
				paths.push_back(SecondHalf);

				replacePortion(lPrev,lNext,paths);

				setSortedIndex();

				if(!getBegin()->equal(*configAtParam(0))){
					cout << "ERROR" << endl;
				}

				if(!getEnd()->equal(*configAtParam(getRangeMax()))){
					getEnd()->print();
					configAtParam(getRangeMax())->print();
					cout << "ERROR" << endl;
				}

				isOptimSuccess = true;

				//				if(!ENV.getBool(Env::debugCostOptim)){
				//					trajToDraw.resize(1);
				//					setColor(0);
				//					trajToDraw.at(0) = *this;
				//					g3d_draw_allwin_active();
				//				}

			}
			if(ENV.getBool(Env::debugCostOptim)){
				//				if(getIdOfPathAt(lCurrent)==getHighestCostId()){
				if(isOptimSuccess)
					debugShowTraj(lPrev,lNext,qNewPt,1);
				else
					debugShowTraj(lPrev,lNext,qNewPt,2);
				//					nbReallyBiased++;
				//				}
			}

		}
		else if(ENV.getBool(Env::debugCostOptim)){
			debugShowTraj(lPrev,lNext,qNewPt,3);
		}

		if(!isOptimSuccess){
			delete FirstHalf;
			delete SecondHalf;
		}
	}

	return isOptimSuccess;
}

void CostOptimization::debugShowTraj(double lPrev,double lNext,shared_ptr<Configuration> qNew, int color ){

	vector< shared_ptr<Configuration> > vectConf(3);
	vectConf.at(0) = configAtParam(lPrev);
	vectConf.at(1) = qNew;
	vectConf.at(2) = configAtParam(lNext);

/*	trajToDraw.resize(4);

	trajToDraw.at(0) = extractSubTrajectory(0,lPrev);
	trajToDraw.at(1) = extractSubTrajectory(lPrev,lNext);
	trajToDraw.at(2) = *new Trajectory(vectConf);
	trajToDraw.at(3) = extractSubTrajectory(lNext,getRangeMax());

	trajToDraw.at(0).setColor(0);
	trajToDraw.at(1).setColor(2);
	trajToDraw.at(2).setColor(color);
	trajToDraw.at(3).setColor(0);*/


	//			basicTraj.print();
	//			triangleTraj.print();
	g3d_draw_allwin_active();

	vector<LocalPath*> pathsTmp(2);
	pathsTmp.at(0) = (new LocalPath(vectConf.at(0),vectConf.at(1)));
	pathsTmp.at(1) = (new LocalPath(vectConf.at(1),vectConf.at(2)));

	Trajectory tmpT(*this);
	tmpT.replacePortion(lPrev,lNext,pathsTmp);
	double newCost = tmpT.cost();

	if( mincost > newCost ){
		mincost = newCost;

	}

	double oldCost = cost();
	double sumOfCost = pathsTmp.at(0)->cost() + pathsTmp.at(1)->cost();
	double costOfPortion = this->costOfPortion(lPrev,lNext);

	cout << "Difference on the portion : "<< (costOfPortion - sumOfCost) << endl;
	cout << "Difference on the trajectory: "<< (oldCost - newCost) << endl;
	cout << "---------------------------------------------------------" << endl;

	double diff = fabs((costOfPortion - sumOfCost) - (oldCost - newCost));

	if(diff > 0.001 ){
		Errors.push_back(diff);
		nbErrors++;
	}

}

void CostOptimization::printDebugInfo(){

	cout << "Errors : " << endl;
	for(uint i=0;i<Errors.size();i++){
		cout << "Errors["<<i<<"] = "<<Errors.at(i)<<endl;
	}

	if(Errors.size()>2){
		double max = *max_element(Errors.begin(),Errors.end());
		double min = *min_element(Errors.begin(),Errors.end());
		cout << "Error Max. =  " << max << endl;
		cout << "Error Min. =  " << min << endl;
	}

	cout << "Selected : " << endl;

	for(uint i=0;i<mSelected.size();i++){
		cout << "mSelected["<<i<<"] = "<< mSelected.at(i)<<endl;
	}
	//	cout << "meanSelected =" <<  meanSelected / mSelected.size() << endl;

	cout << "nbBiased = " << nbBiased << endl;
	cout << "nbReallyBiased = " << nbReallyBiased << endl;
}

int nb_runs = 0;

void CostOptimization::removeRedundantNodes(){

	this->cutTrajInSmallLP();

	uint NbNodes = getNbPaths();

	for(uint i=0; i<NbNodes; i++){
		getLocalPathPtrAt(i)->cost();
	}

	uint initNbNodes = NbNodes;
	uint nbRemoved(0);

	for(uint i=0; i<NbNodes-2; i++){
		for(uint j=i+2; j<NbNodes; j++){

			shared_ptr<Configuration> start = getLocalPathPtrAt(i)->getBegin();
			shared_ptr<Configuration> end 	= getLocalPathPtrAt(j)->getBegin();

			//			cout << "Run : " << nb_runs++ <<  endl;
			LocalPath* pathPtr = new LocalPath(start,end);

			if(pathPtr->getValid()){

				double costOfPortion = 0;

				for(uint k=i;k<j;k++){
					costOfPortion +=  getLocalPathPtrAt(k)->cost();
				}

				if( pathPtr->cost() < costOfPortion ){
					vector<LocalPath*> path;
					path.push_back(pathPtr);
					this->replacePortion(i,j,path);
					NbNodes = getNbPaths();
					nbRemoved++;
				}
				else{
					delete pathPtr;
				}
			}
			else{
				delete pathPtr;
			}
		}
	}

	setSortedIndex();

	cout << nbRemoved << " nodes were removed out of " << initNbNodes << endl;
}

vector< shared_ptr<Configuration> > CostOptimization::getClosestConfOnTraj(
		double& prevDistPt,
		double& randDistPt,
		double& nextDistPt,
		shared_ptr<Configuration> ptrConf,
		double step){

	double delta = this->getRangeMax() / 30 ;

	vector< shared_ptr<Configuration> > vectConf = getNConfAtParam(delta);
	vector< shared_ptr<Configuration> > threeConfVect;

	if(vectConf.size()==0){
		return threeConfVect;
	}

	if(vectConf.size()==1){
		threeConfVect.push_back(vectConf.at(0));
		return threeConfVect;
	}

	// Compute closest configuration on the trajectory
	double minDist = ptrConf->dist(*vectConf.at(0),GENERAL_CSPACE_DIST);
	uint id=0;

	for(uint i=1;i<vectConf.size();i++){

		double dist = ptrConf->dist(*vectConf.at(i),GENERAL_CSPACE_DIST);

		if(  dist < minDist ){
			minDist = dist;
			id = i;
		}
	}

	// Watch out for extreme values on the trajectory
	threeConfVect.resize(3);

	if(id==0){
		id = id + 1;
	}
	if(id==vectConf.size()-1){
		id = id - 1;
	}

	// Compute the parameter of the 3 configurations on the trajectory
	randDistPt = delta*(id);

	int nbOfDelta = (int)((step/delta)+0.5);

	int idPrev = id-nbOfDelta;
	if( idPrev < 0 )
		idPrev = 0;

	int idNext = id+nbOfDelta;
	if( idNext > (vectConf.size()-1) )
		idNext = vectConf.size()-1;

	step = ((double)nbOfDelta)*delta;

	prevDistPt = randDistPt-step;
	if( prevDistPt < 0 )
		prevDistPt = 0;

	nextDistPt = randDistPt+step;
	if( nextDistPt > getRangeMax() )
		nextDistPt = getRangeMax();

	if(ENV.getBool(Env::debugCostOptim)){
		cout << "nbOfDelta = " << nbOfDelta << endl;
		cout << "step = " << step << endl;
		cout << "getRangeMax() = " << getRangeMax() << endl;
		cout << "prevDistPt = " << prevDistPt << endl;
		cout << "randDistPt = " << randDistPt << endl;
		cout << "nextDistPt = " << nextDistPt << endl;
	}

	threeConfVect.at(0) = vectConf.at(idPrev);
	threeConfVect.at(1) = vectConf.at(id);
	threeConfVect.at(2) = vectConf.at(idNext);

	return threeConfVect;

}

vector< shared_ptr<Configuration> > CostOptimization::get3RandSuccesConfAlongTraj(
		double& prevDist,
		double& randDist,
		double& nextDist,
		double step) {

	vector< shared_ptr<Configuration> > vectConf(3);

	if(DeformBiased){
		randDist = getBiasedParamOnTraj();
	}
	else{
		randDist = p3d_random(0,getRangeMax());
	}
	prevDist = MAX(0,randDist - step/2 );
	nextDist = MIN(getRangeMax(), randDist + step/2);

	vectConf.at(0) = configAtParam(prevDist);
	vectConf.at(1) = configAtParam(randDist);
	vectConf.at(2) = configAtParam(nextDist);

	if(prevDist > nextDist ){
		cout << "oqwidjwoqdji" << endl;
	}

	return vectConf;
}
