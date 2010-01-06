#include "HRICS_Cell.h"
#include "Hri_planner-pkg.h"
#include "../HRICS_Planner.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

Cell::Cell() :
        _Open(false),
        _Closed(false)
{

}

Cell::Cell(int i, Vector3i coord , Vector3d corner, Grid* grid) :
        API::Cell(i,corner,grid),
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
    Vector3d center = getCenter();

    //    int x = (int)((center[0]-INTERPOINT->realx)/INTERPOINT->pace);
    //    int y = (int)((center[1]-INTERPOINT->realy)/INTERPOINT->pace);
    //    int z = (int)((center[2]-INTERPOINT->realz)/INTERPOINT->pace);

    double cost;

    if( ENV.getInt(Env::hriCostType) == 0 )
    {
        //        cost = (ENV.getDouble(Env::Kdistance) * hri_exp_distance_val(INTERPOINT,x,y,z));

        Robot* rob = dynamic_cast<Grid*>(this->_grid)->getRobot();

        shared_ptr<Configuration> configStored = rob->getCurrentPos();
        shared_ptr<Configuration> config = rob->getCurrentPos();

        Vector3d cellCenter = this->getCenter();

        config->getConfigStruct()[VIRTUAL_OBJECT+0] = cellCenter[0];
        config->getConfigStruct()[VIRTUAL_OBJECT+1] = cellCenter[1];
        config->getConfigStruct()[VIRTUAL_OBJECT+2] = cellCenter[2];
//        config->getConfigStruct()[VIRTUAL_OBJECT+3] =   0;
//        config->getConfigStruct()[VIRTUAL_OBJECT+4] =   0;
//        config->getConfigStruct()[VIRTUAL_OBJECT+5] =   0;

        rob->setAndUpdate(*config);
        cost = ENV.getDouble(Env::Kdistance)*(HRICS_MOPL->getDistance()->getDistToZones()[0]);
        rob->setAndUpdate(*configStored);
    }

    if( ENV.getInt(Env::hriCostType) == 2 )
    {
        //        cost =  (ENV.getDouble(Env::Kvisibility) * hri_exp_vision_val(INTERPOINT,x,y,z));
    }

    if( ENV.getInt(Env::hriCostType) == 3 )
    {
        //        cost =  (hri_exp_path_val(INTERPOINT,x,y,z));
    }

    return cost;
}


double Cell::getHRICostSpace()
{
    if(!_CostIsComputed)
    {
        Vector3d cellCenter = getCenter();

        //        int x = (int)((center[0]-INTERPOINT->realx)/INTERPOINT->pace);
        //        int y = (int)((center[1]-INTERPOINT->realy)/INTERPOINT->pace);
        //        int z = (int)((center[2]-INTERPOINT->realz)/INTERPOINT->pace);

        if( ENV.getInt(Env::hriCostType) == 0 )
        {
            //            _Cost = hri_exp_distance_val(INTERPOINT,x,y,z);
            if(ENV.getBool(Env::hriCsMoPlanner))
            {
                Robot* rob = HRICS_MOPL->getActivRobot();

                shared_ptr<Configuration> config(new Configuration(rob));

                config->getConfigStruct()[VIRTUAL_OBJECT+0] = cellCenter[0];
                config->getConfigStruct()[VIRTUAL_OBJECT+1] = cellCenter[1];
                config->getConfigStruct()[VIRTUAL_OBJECT+2] = cellCenter[2];
//                config->getConfigStruct()[VIRTUAL_OBJECT+3] =   0;
//                config->getConfigStruct()[VIRTUAL_OBJECT+4] =   0;
//                config->getConfigStruct()[VIRTUAL_OBJECT+5] =   0;

                rob->setAndUpdate(*config);

                _Cost = HRICS_MOPL->getDistance()->getDistToZones()[0];
                //           cout << "Get cost of (" << center[0] << " , " << center[1] << " , " << center[2] << " ) " << endl;
                //           cout << "_Cost = " << _Cost << endl;
            }
        }

        if( ENV.getInt(Env::hriCostType) == 2 )
        {
            //            _Cost = hri_exp_vision_val(INTERPOINT,x,y,z);
        }

        if( ENV.getInt(Env::hriCostType) == 3 )
        {
            //            _Cost = hri_exp_path_val(INTERPOINT,x,y,z);
        }

        _CostIsComputed = true;
    }

    return _Cost;
}

void Cell::resetExplorationStatus()
{
    //    cout << "Reseting Cell " << this << endl;
    _Open = false;
    _Closed =false;
}

void Cell::drawCell()
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