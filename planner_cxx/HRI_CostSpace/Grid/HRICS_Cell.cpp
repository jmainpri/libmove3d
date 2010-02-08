#include "HRICS_Cell.h"
#include "Hri_planner-pkg.h"

#include "../HRICS_Planner.h"
#include "../HRICS_CSpace.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

Cell::Cell() :
        _Open(false),
        _Closed(false)
{

}

Cell::Cell(int i, Vector3i coord , Vector3d corner, Grid* grid) :
        API::ThreeDCell(i,corner,grid),
        _Open(false),
        _Closed(false),
        _CostIsComputed(false)
{
    _Coord = coord;
    _v0 = new double[3]; _v1 = new double[3]; _v2 = new double[3]; _v3 = new double[3];
    _v4 = new double[3]; _v5 = new double[3]; _v6 = new double[3]; _v7 = new double[3];

    _v0[0] = _corner[0] + _grid->getCellSize()[0];
    _v0[1] = _corner[1] + _grid->getCellSize()[1];
    _v0[2] = _corner[2] + _grid->getCellSize()[2];

    _v1[0] = _corner[0] ;
    _v1[1] = _corner[1] + _grid->getCellSize()[1];
    _v1[2] = _corner[2] + _grid->getCellSize()[2];

    _v2[0] = _corner[0] ;
    _v2[1] = _corner[1] ;
    _v2[2] = _corner[2] + _grid->getCellSize()[2];

    _v3[0] = _corner[0] + _grid->getCellSize()[0];
    _v3[1] = _corner[1] ;
    _v3[2] = _corner[2] + _grid->getCellSize()[2];

    _v4[0] = _corner[0] + _grid->getCellSize()[0];
    _v4[1] = _corner[1] ;
    _v4[2] = _corner[2] ;

    _v5[0] = _corner[0] + _grid->getCellSize()[0];
    _v5[1] = _corner[1] + _grid->getCellSize()[1];
    _v5[2] = _corner[2] ;

    _v6[0] = _corner[0] ;
    _v6[1] = _corner[1] + _grid->getCellSize()[1];
    _v6[2] = _corner[2] ;

    _v7[0] = _corner[0] ;
    _v7[1] = _corner[1] ;
    _v7[2] = _corner[2] ;

}

double Cell::getCost()
{
    if(_CostIsComputed && (!ENV.getBool(Env::RecomputeCellCost)))
    {
        return _Cost;
    }
    Robot* rob = dynamic_cast<Grid*>(this->_grid)->getRobot();

    shared_ptr<Configuration> configStored = rob->getCurrentPos();
    shared_ptr<Configuration> config = rob->getCurrentPos();

    Vector3d cellCenter = this->getCenter();

    config->getConfigStruct()[rob->getObjectDof()+0] = cellCenter[0];
    config->getConfigStruct()[rob->getObjectDof()+1] = cellCenter[1];
    config->getConfigStruct()[rob->getObjectDof()+2] = cellCenter[2];

    rob->setAndUpdate(*config);

    double cost;
    if(ENV.getBool(Env::HRIPlannerWS))
    {
        switch ( ENV.getInt(Env::hriCostType))
        {
        case 0 :
            _Cost = ENV.getDouble(Env::Kdistance)*(HRICS_MOPL->getDistance()->getDistToZones()[0]);
            break;
        case 1 :
            break;
        case 2 :
            break;
        default:
            cout << "Type of Cost undefine in Grid "  << endl;
        }
    }

    if(ENV.getBool(Env::HRIPlannerCS))
    {
        switch ( ENV.getInt(Env::hriCostType))
        {
        case 0 :
            _Cost = HRICS_CSpaceMPL->getDistanceCost();
            break;
        case 1 :
            _Cost = HRICS_CSpaceMPL->getVisibilityCost(cellCenter);
            break;
        case 2 :
            break;
        default:
            cout << "Type of Cost undefine in Grid "  << endl;
        }
    }

    _CostIsComputed = true;
    rob->setAndUpdate(*configStored);
    return _Cost;
}

/**
  * Hope fully unused
  */
//double Cell::getHRICostSpace()
//{
//    double costDistance = 0.0;
//    double costVisibility = 0.0;
//
//    if( (!_CostIsComputed) || ENV.getBool(Env::HRIPlannerCS) )
//    {
//        Vector3d cellCenter = getCenter();
//
//        if( (ENV.getInt(Env::hriCostType) == 0) || (ENV.getInt(Env::hriCostType) == 3) )
//        {
//            Robot* rob = dynamic_cast<Grid*>(_grid)->getRobot();
//
//            shared_ptr<Configuration> config(new Configuration(rob));
//
//            config->getConfigStruct()[rob->getObjectDof()+0] = cellCenter[0];
//            config->getConfigStruct()[rob->getObjectDof()+1] = cellCenter[1];
//            config->getConfigStruct()[rob->getObjectDof()+2] = cellCenter[2];
//
//            rob->setAndUpdate(*config);
//
//            if(ENV.getBool(Env::HRIPlannerWS))
//            {
//                //                cout << "VIRTUAL_OBJECT_DOF  = "  << VIRTUAL_OBJECT_DOF << endl;
//                //                cout << "costDistance  = " << costDistance << endl;
//                costDistance = HRICS_MOPL->getDistance()->getDistToZones()[0];
//            }
//            else
//            {
//                if(ENV.getBool(Env::HRIPlannerCS))
//                {
//                    costDistance = HRICS_CSpaceMPL->getDistanceCost();
//                }
//            }
//        }
//
//        if( (ENV.getInt(Env::hriCostType) == 1 ) || (ENV.getInt(Env::hriCostType) == 3) )
//        {
//            costVisibility = HRICS_CSpaceMPL->getVisibilityCost(cellCenter);
//        }
//
//        _Cost = ENV.getDouble(Env::Kdistance)*costDistance + ENV.getDouble(Env::Kvisibility)*costVisibility ;;
//        _CostIsComputed = true;
//    }
//    return _Cost;
//}

void Cell::resetExplorationStatus()
{
    //    cout << "Reseting Cell " << this << endl;
    _Open = false;
    _Closed =false;
}

void Cell::draw()
{
    glNormal3f(0,0,1);
    glVertex3dv(_v0);    // front face
    glVertex3dv(_v1);
    glVertex3dv(_v2);
    glVertex3dv(_v3);

    glNormal3f(1,0,0);
    glVertex3dv(_v0);    // right face
    glVertex3dv(_v3);
    glVertex3dv(_v4);
    glVertex3dv(_v5);

    glNormal3f(0,1,0);
    glVertex3dv(_v0);    // up face
    glVertex3dv(_v5);
    glVertex3dv(_v6);
    glVertex3dv(_v1);

    glNormal3f(-1,0,0);
    glVertex3dv(_v1);
    glVertex3dv(_v6);
    glVertex3dv(_v7);
    glVertex3dv(_v2);

    glNormal3f(0,0,-1);
    glVertex3dv(_v4);
    glVertex3dv(_v7);
    glVertex3dv(_v6);
    glVertex3dv(_v5);

    glNormal3f(0,-1,0);
    glVertex3dv(_v7);
    glVertex3dv(_v4);
    glVertex3dv(_v3);
    glVertex3dv(_v2);
}
