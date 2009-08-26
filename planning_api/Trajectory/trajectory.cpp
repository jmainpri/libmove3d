/*
 * trajectory.cpp
 *
 *  Created on: Jun 17, 2009
 *      Author: jmainpri
 */

#include "trajectory.hpp"

using namespace std;
using namespace tr1;

Trajectory::Trajectory():
	mRobot(NULL),
	nloc(0),
	name(""),
	file(""),
	range_param(0),
	color(0),
	HighestCostId(0),
	isHighestCostIdSet(false) {
}

Trajectory::Trajectory(Robot* R) :
	mRobot(R),
	nloc(0),
	name(""),
	file(""),
	range_param(0),
	color(0),
	HighestCostId(0),
	isHighestCostIdSet(false){
}

Trajectory::Trajectory(const Trajectory& T) :
	name(T.name),
	file(T.file),
	mRobot(T.mRobot),
	nloc(T.nloc),
	range_param(T.range_param),
	mBegin(T.mBegin),
	mEnd(T.mEnd),
	color(T.color),
	HighestCostId(T.HighestCostId),
	isHighestCostIdSet(T.isHighestCostIdSet){

	for(uint i=0;i<nloc;i++){
		mCourbe.push_back(new LocalPath(*(T.mCourbe.at(i))));
	}
//	cout << "Copy Trajectory" << endl;
}

Trajectory& Trajectory::operator= (const Trajectory& t){

	if(nloc>0){
		for(uint i=0;i<nloc;i++){
			delete mCourbe.at(i);
		}
	}

	mCourbe.clear();

	// TODO Name of file and robot
	name = t.name;
	file = t.file;

	mRobot = t.mRobot;

	/* Number of localpath */
	nloc = t.nloc;
	range_param = t.range_param;

	for(uint i=0;i<nloc;i++){
		mCourbe.push_back(new LocalPath(*(t.mCourbe.at(i))));
	}

//	cout << "Copy in operator= Trajtecory" << endl;

	mBegin = t.mBegin;
	mEnd = t.mEnd;
	color = t.color;
	HighestCostId = t.HighestCostId;
	isHighestCostIdSet= t.isHighestCostIdSet;

	return *this;
}

Trajectory::Trajectory(std::vector< shared_ptr<Configuration> > C):
	HighestCostId(0),
	isHighestCostIdSet(false){

	if(C.size()>1){
		name = "";
		file = "";

		nloc = C.size()-1;        /* Number of localpath */
		mRobot = C.at(0)->getRobot();
		range_param = 0;

		mBegin = C.at(0);
		mEnd = C.at(nloc);

		for(uint i=0;i<nloc;i++){
			LocalPath* path = new LocalPath(C.at(i),C.at(i+1));
			range_param += path->getParamMax();
			mCourbe.push_back(path);
		}
	}
	else{
		cout << "Warning: Constructing a class out of empty vector of configuration" << endl;
	}
	color = 1;
}

Trajectory::Trajectory(Robot* R,p3d_traj* t){

	// TODO Name and file (string based)

	mRobot = R;

	nloc = t->nlp;

	p3d_localpath *localpathPt;

	if(t!= NULL){

		localpathPt = t->courbePt;

		while (localpathPt != NULL){
			LocalPath* path = new LocalPath(mRobot,localpathPt);
//			path->getBegin()->print();
//			path->getEnd()->print();
			mCourbe.push_back(path);
			localpathPt = localpathPt->next_lp;
		}
		range_param = p3d_compute_traj_rangeparam(t);
//		cout << "range_param = " << range_param << endl;
	}

	mBegin = shared_ptr<Configuration>(new Configuration(
			mRobot,
			p3d_config_at_param_along_traj(t,0)));

//	cout << "mBegin:" << endl;
//	mBegin->print();

	mEnd = shared_ptr<Configuration>(new Configuration(
			mRobot,
			p3d_config_at_param_along_traj(t,range_param)));

//	cout << "mEnd:" << endl;
//	mEnd->print();

	updateRange();
//	cout << "range_param = " << range_param << endl;
	color = 0;
}

Trajectory::~Trajectory(){

//	cout << "Delete Trajectory ("<<nloc<<")" << endl;

	for(uint i=0;i<nloc;i++){
//		cout<<"delete "<<i<<endl;
		delete mCourbe.at(i);
	}
}

void Trajectory::replaceP3dTraj(){

	replaceP3dTraj(mRobot->getTrajStruct());

}

void Trajectory::replaceP3dTraj(p3d_traj* trajPt){

	//	print();

	destroy_list_localpath(mRobot->getRobotStruct(),trajPt->courbePt);

	if (!trajPt){
		cout << "Error : Allocation problem in trajectory replaceP3dTraj" << endl;
		return;
	}

	//	trajPt->name       = strdup(name);
	//	trajPt->file       = NULL;  // Modification Fabien
	trajPt->num        = 0; //mRobot->getRobotStruct()->nt;
	trajPt->rob        = mRobot->getRobotStruct();

	//	cout << mRobot->getRobotStruct() << endl;

	nloc = (int)mCourbe.size();
	//	cout << "Number of local paths : " << nloc << endl;

	p3d_localpath *localpathPt = mCourbe.at(0)->getLocalpathStruct()->copy(mRobot->getRobotStruct(),mCourbe.at(0)->getLocalpathStruct());
	p3d_localpath *localprevPt = NULL;

	localpathPt->prev_lp = localprevPt;
	trajPt->courbePt = localpathPt;

	for(uint i=1;i<nloc;i++){

		localprevPt = localpathPt;

		localpathPt = mCourbe.at(i)->getLocalpathStruct()->copy(mRobot->getRobotStruct(),mCourbe.at(i)->getLocalpathStruct());
		//		cout << "localpathPt = " << localpathPt << endl;

		localprevPt->next_lp = localpathPt;
		//		cout << "localprevPt->next_lp = " << localpathPt << endl;

		localpathPt->prev_lp = localprevPt;
		//		cout << "localpathPt->prev_lp = " << localpathPt << endl;

		if(i!=nloc-1){
			localpathPt->next_lp = mCourbe.at(i+1)->getLocalpathStruct();
			//			cout << "localpathPt->next_lp = " << mCourbe.at(i+1)->getLocalpathStruct() << endl;
		}
	}

	localpathPt->next_lp = NULL;

	trajPt->nlp = nloc;

	trajPt->range_param = p3d_compute_traj_rangeparam(trajPt);

	//	print()

}

shared_ptr<Configuration> Trajectory::configAtParam(double param){

	double soFar(0.0);
	double prevSoFar(0.0);

	for(uint i=0;i<nloc;i++){

		soFar = soFar + mCourbe.at(i)->getParamMax();

		// Parameter lies in the previous local path
		// return configuration inside previous LP
		if(param < soFar){

			if(param < prevSoFar){
				cout << "Error: getting Configuration at parameter in trajectory" << endl;
			}

			return mCourbe.at(i)->configAtParam(param-prevSoFar);
		}
		prevSoFar = soFar;
	}

	return mCourbe.back()->configAtParam(param);
}

vector< shared_ptr<Configuration> > Trajectory::getNConfAtParam(double delta){

	vector< shared_ptr<Configuration> > tmpVector(0);

	double param=0;

	double soFar(0.0);
	double prevSoFar(0.0);

	for(uint i=0;i<nloc;i++){

			soFar = soFar + mCourbe.at(i)->getParamMax();

			// Parameter lies in the previous local path
			// return configuration inside previous LP
			while(param < soFar){

				if(param < prevSoFar){
					cout << "Error: getting Configuration at parameter in trajectory" << endl;
				}

				tmpVector.push_back(mCourbe.at(i)->configAtParam(param-prevSoFar));
				param += delta;
			}
			// TODO watch out
			if(i<nloc-1){
				prevSoFar = soFar;
			}
		}

		tmpVector.push_back(mCourbe.at(nloc-1)->configAtParam(param-prevSoFar));

		return tmpVector;

}

LocalPath* Trajectory::getLocalPathPtrAt(uint id){
	return mCourbe.at(id);
}

int Trajectory::getNbPaths(){
	return mCourbe.size();
}

double Trajectory::computeSubPortionRange(vector<LocalPath*> portion){

	double range(0.0);

	for(uint i=0;i<portion.size();i++){
		range += portion[i]->getParamMax();
	}

	return range;
}

void Trajectory::updateRange(){

	nloc =  mCourbe.size();
	range_param = computeSubPortionRange(mCourbe);

}

double Trajectory::computeSubPortionCost(vector<LocalPath*> portion){

	double cost(0.0);

	for(uint i=0;i<portion.size();i++){
		cost += portion[i]->cost();
	}

	return cost;
}

double Trajectory::cost(){

	double cost(0.0);

	cost = computeSubPortionCost(mCourbe);

	//	cout << "Traj cost = " << cost << " for " << nloc << " localpaths"<< endl;

	return cost;
}

vector<LocalPath*> Trajectory::extractSubPortion(double param1,double param2,uint& first,uint& last){

	vector<LocalPath*> paths;

	if( param1 > param2 ){
		cout << "Warning: Error not possible to replace trajectory because of the parameters" << endl;
		return paths;
	}

	// Looks for the local paths to be changed linear in NLOC
	//-------------------------------------------------------------------
	double soFar(0.0);
	double prevSoFar(0.0);

	shared_ptr<Configuration> q1;
	shared_ptr<Configuration> q2;

	for(uint i=0;i<mCourbe.size();i++){

		soFar = soFar + mCourbe.at(i)->getParamMax();

		if(param1 < soFar){
			q1	= mCourbe.at(i)->configAtParam(param1-prevSoFar);
//			q1->print();
//			param_start = param1-prevSoFar;
			first = i;
			break;
		}
		prevSoFar = soFar;
		if(i==(nloc-1)){
//			cout << "Warning: cannont build a traj with same goal and start" << endl;
//			return paths;
			q1	= mCourbe.at(i)->getEnd();
			first = i;
		}
	}

	// id_LP_1 is the id of the path of witch param1 lies in
	// we start searching for parm2 from here
	soFar = prevSoFar;

	for(uint i=first;i<mCourbe.size();i++){

		soFar = soFar + mCourbe.at(i)->getParamMax();

		if(param2 < soFar){
			q2	= mCourbe.at(i)->configAtParam(param2-prevSoFar);
//			q2->print();
//			param_end = param2-soFar;
			last = i;
			break;
		}
		prevSoFar = soFar;

		if(i==(nloc-1)){
			q2 = mCourbe.at(nloc-1)->getEnd();
			last = i;
		}
	}


	// Adds to paths the portion of local path (At begin and End of the portion)
	//----------------------------------------------------------------------------

//	cout << "Number of LP = "<< last-first+1 <<endl;
//	cout << "This traj nb LP = " << mCourbe.size() << endl;

	if(first==last){
		if(q1->equal(*q2)){
			paths.resize(0);
			cout << "Trajectory null" << endl;
			return paths;
		}
		paths.push_back(new LocalPath(q1,q2));
		return paths;
	}

	if( first > last ){
		cout << "ERROR: inconsistent query for subpath extraction" << endl;
		return paths;
	}

	shared_ptr<Configuration> start = mCourbe.at(first)->getBegin();
	shared_ptr<Configuration> end = mCourbe.at(last)->getEnd();

	// Adds the modified first local path to subpaths
	// Verifies that the configuration is not starting the local path
	if(!start->equal(*q1)){

		if(!mCourbe.at(first)->getEnd()->equal(*q1)){

			LocalPath* startNew = new LocalPath(q1,mCourbe.at(first)->getEnd());

			if(startNew->getValid()){
				paths.push_back(startNew);
			}
			else{
				cout << "Error: portion of path not valid" << endl;
				return paths;
			}
		}
	}
	else{
		LocalPath* startNew = new LocalPath(*mCourbe.at(first));

		if(startNew->getValid()){
			paths.push_back(startNew);
		}
	}

	// Adds all the paths between First and Last
	for(uint i=first+1;i<last;i++){
		paths.push_back(new LocalPath(*mCourbe.at(i)));
	}

	// Verifies that the configuration is not ending the local path
	// Adds the modified Last local path to subpaths
	if(!end->equal(*q2)){

		if(!mCourbe.at(last)->getBegin()->equal(*q2)){

			LocalPath* endNew = new LocalPath(mCourbe.at(last)->getBegin(),q2);

			if(endNew->getValid()){
				paths.push_back(endNew);
			}
			else{
				cout << "Error: portion of path not valid" << endl;
				return paths;
			}
		}
	}
	else{
		LocalPath* endNew = new LocalPath(*mCourbe.at(last));

		if(endNew->getValid()){
			paths.push_back(endNew);
		}
	}

	// Verifies the integrity of the sub paths
	if( (!q1->equal(*paths.at(0)->getBegin())) || (!q2->equal(*paths.back()->getEnd()))){
		paths.at(0)->getBegin()->print();
		q1->print();
		paths.back()->getEnd()->print();
		q2->print();
		cout << "ERROR in extract sub portion integrity"<< endl;
	}

	return paths;
}


Trajectory Trajectory::extractSubTrajectory(double param1, double param2){

	uint first(0);
	uint last(0);

	vector<LocalPath*> path = extractSubPortion(
			param1,param2,
			first,last);

	Trajectory newTraj(mRobot);

	newTraj.mCourbe = path;
	newTraj.nloc = path.size();

	if(path.size()==0){
		newTraj.mBegin = configAtParam(param1);
		newTraj.mEnd = newTraj.mBegin;
		newTraj.range_param = 0;
	}
	else{
		newTraj.mBegin = path.at(0)->getBegin();
		newTraj.mEnd = path.back()->getEnd();
		newTraj.range_param = computeSubPortionRange(path);
	}

	return newTraj;
}

extern double ZminEnv;
extern double ZmaxEnv;

extern void* GroundCostObj;

void Trajectory::drawGL(int nbKeyFrame){

	double du = range_param / nbKeyFrame;
	double u=du;

	int val1, val2;
	double Cost1, Cost2;

	p3d_obj *o;

	int NumBody = mRobot->getRobotStruct()->no - 1;

	if( ( NumBody >= mRobot->getRobotStruct()->no)||( NumBody < 0 ))
		return;

	if(!(o = mRobot->getRobotStruct()->o[NumBody]))
		return;

	shared_ptr<Configuration> qSave = mRobot->getCurrentPos();
	shared_ptr<Configuration> q = mBegin;
	mRobot->setAndUpdate(*q);

	p3d_vector3 pi,pf;
	p3d_jnt_get_cur_vect_point(o->jnt, pi);

	int saveColor;
	bool red=false;

	while( u < range_param )
	{
		/* position of the robot corresponding to parameter u */
		q = configAtParam(u);
		mRobot->setAndUpdate(*q);
		p3d_jnt_get_cur_vect_point(o->jnt,pf);

		if(isHighestCostIdSet){
			if(getIdOfPathAt(u)==HighestCostId && !red){
				red = true;
				saveColor = color;
				color = 3;
			}
			if( (color == 3) && (getIdOfPathAt(u)!=HighestCostId) && red ){
				color = saveColor;
				red = false;
			}
		}


		if((!ENV.getBool(Env::isCostSpace)) || (GroundCostObj == NULL)) {
			glLineWidth(3.);
			g3d_drawOneLine(pi[0],pi[1],pi[2],pf[0],pf[1],pf[2],color,NULL);
			glLineWidth(1.);
		} else {
			val1 = GHintersectionVerticalLineWithGround(GroundCostObj, pi[0], pi[1], &Cost1);
			val2 = GHintersectionVerticalLineWithGround(GroundCostObj, pf[0], pf[1], &Cost2);
			glLineWidth(3.);
			g3d_drawOneLine(pi[0],pi[1],Cost1+(ZmaxEnv-ZminEnv)*0.02,pf[0],pf[1],Cost2+(ZmaxEnv-ZminEnv)*0.02,color,NULL);
			glLineWidth(1.);
		}
		p3d_vectCopy(pf, pi);
		u += du;
	}

	mRobot->setAndUpdate(*qSave);
}

double Trajectory::costOfPortion(double param1,double param2){

	uint first(0);
	uint last(0);

	vector<LocalPath*> path = extractSubPortion(param1,param2,first,last);

	double Cost = computeSubPortionCost(path);

	for(int i=0;i<path.size();i++)
	{
		delete path.at(i);
	}
	return Cost;

}
//double Trajectory::costOfPortion(double param1,double param2){
//
//	double soFar(0.0);
//	double prevSoFar(0.0);
//	uint id_start;
//	uint id_end;
//	shared_ptr<Configuration> confPtrStart;
//	shared_ptr<Configuration> confPtrEnd;
//
//	// TODO change that function
//
//	if( param1 > param2 ){
//		cout << "Error: in Trajectory::costofPortion() " << endl;
//		return 0;
//	}
//
//	for(uint i=0;i<nloc;i++){
//
//		soFar = soFar + mCourbe.at(i)->getParamMax();
//
//		// Parameter lies in the previous local path
//		// return configuration inside previous LP
//		if(param1 < soFar){
//
//			if(param1 < prevSoFar)
//				cout << "Error getting Conf at param" << endl;
//
//			confPtrStart = mCourbe.at(i)->configAtParam(param1-prevSoFar);
//			id_start = i;
//			break;
//		}
//		if(i==nloc-1){
//			confPtrStart = mCourbe.at(i)->configAtParam(param1-prevSoFar);
//			id_start = i;
//		}
//		prevSoFar = soFar;
//	}
//
//	soFar = prevSoFar;
//
//	for(uint i=id_start;i<nloc;i++){
//
//		soFar = soFar + mCourbe.at(i)->getParamMax();
//
//		// Paramameter lies in the previous local path
//		// return configuration inside previous LP
//		if(param2 < soFar){
//
//			if(param2 < prevSoFar)
//				cout << "Error getting Conf at param" << endl;
//
//			confPtrEnd = mCourbe.at(i)->configAtParam(param2-prevSoFar);
//			id_end = i;
//			break;
//		}
//		if(i==nloc-1){
//			confPtrEnd = mCourbe.at(i)->getEnd();
//			id_end = i;
//		}
//		prevSoFar = soFar;
//	}
//
//	LocalPath pathStart(confPtrStart, mCourbe.at(id_start)->getEnd());
//	LocalPath pathEnd(mCourbe.at(id_end)->getBegin(),confPtrEnd);
//
//	double cost =0.0;
//
//	cost += pathStart.cost() + pathEnd.cost();
//
//	for(uint i=id_start+1;i<id_end;i++){
//		cost+=mCourbe.at(i)->cost();
//	}
//
//	return cost;
//}

uint Trajectory::cutPortionInSmallLP(vector<LocalPath*>& portion){

	uint nLP = 40;
	double range = computeSubPortionRange(portion);
	double dMax = range/40;

	uint j 				= 0;
	double soFar 		= 0.0;
	double prevSoFar 	= 0.0;
	double param		= 0.0;

	if(nLP==0){
		cout << "ERROR: cutPortionInSmallLP" << endl;
	}

	if(nLP==1){
		return nLP;
	}

	vector<LocalPath*> portionTmp(nLP);
	shared_ptr<Configuration> confPtr;
	shared_ptr<Configuration> confPtrTmp;

	prevSoFar = 0.0;
	soFar = portion.at(0)->getParamMax();
	confPtrTmp = portion.at(0)->getBegin();

	// Loop for every Small LP
	for(uint i=0;i<nLP-1;i++){

		param = param + dMax;

		// Check to go seek the conf on next LP
		while( (param > soFar) && (j<portion.size()) ){
			j = j + 1;
			prevSoFar = soFar;
			soFar = soFar + portion.at(j)->getParamMax();
		}

		confPtr = portion.at(j)->configAtParam( param - prevSoFar );
		portionTmp.at(i) = new LocalPath(confPtrTmp,confPtr);
		confPtrTmp = confPtr;
	}

	// The last LP is made of the end configuration
	confPtr 		= portion.back()->getEnd();
	confPtrTmp		= portionTmp.at(nLP-2)->getEnd();

	portionTmp.back() =  new LocalPath(confPtrTmp,confPtr);

	// Delete and replace every thing
	for(uint i=0;i<portion.size();i++){
		delete portion.at(i);
	}

	portion.clear();
	portion = portionTmp;

	if(nLP != portion.size()){
		cout << "ERROR: cutPortionInSmallLP" << endl;
	}

	return nLP;
}

void Trajectory::cutTrajInSmallLP(){

	uint nLP = cutPortionInSmallLP(mCourbe);
	updateRange();

	cout << "Cutting into " << nLP << " local paths" << endl;
	cout << "Traj Size = " << nloc << endl;
	cout << "Cost Of trajectory :" << this->cost() << endl;

	if(!mBegin->equal(*configAtParam(0))){
		cout << "ERROR" << endl;
	}

	if(!mEnd->equal(*configAtParam(range_param))){
		mEnd->print();
		configAtParam(range_param)->print();
		cout << "ERROR" << endl;
	}

}

void Trajectory::replacePortion( uint id1, uint id2, vector<LocalPath*> paths){

	// WARNING: the ids are ids of nodes and not
	// LocalPaths

	if( id1 < id2 )
	{
		for( uint i = id1; i < id2; i++ )
		{
			delete mCourbe.at(i);
		}

		mCourbe.erase(
				mCourbe.begin()+id1,
				mCourbe.begin()+id2);
	}
	if(id1 == id2){
		cout << "Error: in replace local" << endl;
	}

	mCourbe.insert(
			mCourbe.begin()+id1,
			paths.begin(),
			paths.end());

	nloc = mCourbe.size();

	updateRange();

}

void Trajectory::replacePortion( double param1, double param2, vector<LocalPath*> paths){

	shared_ptr<Configuration> q11 = paths.at(0)->getBegin();
	shared_ptr<Configuration> q12 = paths.back()->getEnd();

	shared_ptr<Configuration> q21;
	shared_ptr<Configuration> q22;

	if(param1 > param2){
		cout << "Warning: Error not possible to replace trajectory because of the parameters" << endl;
		return;
	}

	// TODO replace with extratSubPortion

	// Looks for the local paths to be changed linear in NLOC
	//-------------------------------------------------------------------
	double soFar(0.0);
	double prevSoFar(0.0);

	double param_start(0.0);
	double param_end(0.0);

	uint id_LP_1(0);
	uint id_LP_2(0);

	for(uint i=0;i<nloc;i++){

		soFar = soFar + mCourbe[i]->getParamMax();

		if(param1 < soFar){
			q21	= mCourbe.at(i)->configAtParam(param1-prevSoFar);
			param_start = param1-prevSoFar;
			id_LP_1 = i;
			break;
		}
		prevSoFar = soFar;
		if(i==(nloc-1)){
//			cout << "Warning: cannont build a traj with same goal and start" << endl;
//			return;
		}
	}

	soFar = prevSoFar;

	for(uint i=id_LP_1;i<nloc;i++){

		soFar = soFar + mCourbe[i]->getParamMax();

		if(param2 < soFar){
			q22	= mCourbe.at(i)->configAtParam(param2-prevSoFar);
			param_end = param2-soFar;
			id_LP_2 = i;
			break;
		}
		prevSoFar = soFar;

		if(i==(nloc-1)){
			q22 = mCourbe.at(nloc-1)->getEnd();
			id_LP_2 = i;
			param_end = soFar;
		}
	}

	// Condition has to be false if the trajectory is consistent
	if((!q11->equal(*q21)) || (!q12->equal(*q22))){
		if(!q11->equal(*q21)) {
			q11->print();
			q21->print();
		}
		if(!q12->equal(*q22)) {
			q12->print();
			q22->print();
		}
		cout << "Warning: Error not possible to replace trajectory because configuration are not the same" << endl;
		return;
	}


	// Adds to paths the portion of local path (At begin and End of the portion)
	//----------------------------------------------------------------------------
	shared_ptr<Configuration> start = mCourbe.at(id_LP_1)->getBegin();
	shared_ptr<Configuration> end = mCourbe.at(id_LP_2)->getEnd();

	// Verifies that the configuration is not starting the local path
	if(!start->equal(*q21)){

		LocalPath* startNew = new LocalPath(start,q21);

		if(startNew->getValid()){
			paths.insert(paths.begin(),new LocalPath(start,q21));
		}
		else{
			cout << "Error: portion of path not valid" << endl;
			return;
		}
	}

	// Verifies that the configuration is not ending the local path
	if(!end->equal(*q22)){

		LocalPath* endNew = new LocalPath(q22,end);

		if(endNew->getValid()){
			paths.push_back(endNew);
		}
		else{
			cout << "Error: portion of path not valid" << endl;
			return;
		}
	}

	//	cout << "PathSizeOf() = " << paths.size() << endl;
	// TODO optional
	//cutPortionInSmallLP(paths);


	// Free, Erases the old ones and the Inserts the new ones
	//---------------------------------------------------------------------------
	uint id1_erase = id_LP_1;
	uint id2_erase = id_LP_2+1;

	replacePortion(id1_erase,id2_erase,paths);
}

uint Trajectory::getIdOfPathAt(double param){

	double soFar(0.0);

	for(uint i=0;i<nloc;i++){

		soFar = soFar + mCourbe.at(i)->getParamMax();

		if(param < soFar){
			return i;
		}
	}
	return nloc-1;
}

void Trajectory::print(){

	cout << "-------------- Trajectory --------------" << endl;
	cout << " Number of LP " << nloc << endl;
	cout << " Range Parameter " << this->range_param << endl;

	for(uint i=0;i<nloc;i++){
		mCourbe.at(i)->print();
	}

	cout << "-----------------------------------" << endl;
}
