#include "HriCell.h"
#include "Hri_planner-pkg.h"
#include "../../p3d/env.hpp"
#include <iostream>

using namespace std;

HriCell::HriCell() :
        _Open(false),
        _Closed(false)
{

}

HriCell::HriCell(int i, vector<int> coord , vector<double> corner, HriGrid* grid) :
        Cell(i,corner,grid),
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

double HriCell::getCost()
{
    vector<double> center = getCenter();

    int x = (int)((center[0]-INTERPOINT->realx)/INTERPOINT->pace);
    int y = (int)((center[1]-INTERPOINT->realy)/INTERPOINT->pace);
    int z = (int)((center[2]-INTERPOINT->realz)/INTERPOINT->pace);

    double cost;

    if( ENV.getInt(Env::hriCostType) == 0 )
    {
        cost = (ENV.getDouble(Env::Kdistance) * hri_exp_distance_val(INTERPOINT,x,y,z));
    }

    if( ENV.getInt(Env::hriCostType) == 2 )
    {
        cost =  (ENV.getDouble(Env::Kvisibility) * hri_exp_vision_val(INTERPOINT,x,y,z));
    }

    if( ENV.getInt(Env::hriCostType) == 3 )
    {
        cost =  (hri_exp_path_val(INTERPOINT,x,y,z));
    }

    return cost;
}


double HriCell::getHRICostSpace()
{
    if(true/*!_CostIsComputed*/)
    {
        vector<double> center = getCenter();

        int x = (int)((center[0]-INTERPOINT->realx)/INTERPOINT->pace);
        int y = (int)((center[1]-INTERPOINT->realy)/INTERPOINT->pace);
        int z = (int)((center[2]-INTERPOINT->realz)/INTERPOINT->pace);

        if( ENV.getInt(Env::hriCostType) == 0 )
        {
            _Cost = hri_exp_distance_val(INTERPOINT,x,y,z);
        }

        if( ENV.getInt(Env::hriCostType) == 2 )
        {
            _Cost = hri_exp_vision_val(INTERPOINT,x,y,z);
        }

        if( ENV.getInt(Env::hriCostType) == 3 )
        {
            _Cost = hri_exp_path_val(INTERPOINT,x,y,z);
        }

        _CostIsComputed = true;
    }

    return _Cost;
}

void HriCell::resetExplorationStatus()
{
//    cout << "Reseting Cell " << this << endl;
    _Open = false;
    _Closed =false;
}

void HriCell::drawCell()
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
