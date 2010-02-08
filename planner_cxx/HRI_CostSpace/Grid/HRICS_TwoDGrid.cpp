#include "HRICS_TwoDGrid.hpp"

using namespace std;
using namespace tr1;
using namespace HRICS;

PlanGrid::PlanGrid() :
        API::TwoDGrid(),
        mRobot(0x00)
{
}

PlanGrid::PlanGrid(double pace, vector<double> envSize) :
        API::TwoDGrid(pace,envSize),
        mRobot(0x00)
{
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY << endl;
}

/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
API::TwoDCell* PlanGrid::createNewCell(int index, int x, int y )
{
    if (index == 0)
    {
        return new PlanCell( 0, _originCorner , this );
    }
    API::TwoDCell* newCell = new PlanCell( index, computeCellCorner(x,y) , this );
//    Vector2d corner = newCell->getCorner();
    //    cout << " = (" << corner[0] <<"," << corner[1] << ")" << endl;
    return newCell;
}

void PlanGrid::draw()
{
    if( mRobot == 0x00 )
    {
        std::cout << "Error : PlanGrid::draw() => No Robot "  << std::endl;
    }

    deactivateCcCntrts(mRobot->getRobotStruct(),-1);

    double colorvector[4];

    colorvector[0] = 1.0;       //red
    colorvector[1] = 0.5;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 0.05;       //transparency


    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);

    double depth = 0.20;

//    cout << "Drawing 2D Grid"  << endl;

    double color[3];

//    double nbCells = static_cast<double>(getNumberOfCells());

    for (int x =0;x<_nbCellsX;++x)
    {
        for (int y =0;y<_nbCellsY;++y)
        {

            PlanCell* Cell = dynamic_cast<PlanCell*>(getCell(x,y));

            double colorRation = 2-Cell->getCost();

            Vector2d center = Cell->getCenter();

//            double colorRation = (((double)x*(double)_nbCellsY)+(double)y)/(nbCells);
//            cout << " X = "  << _nbCellsX << " , Y = "  << _nbCellsY << endl;
//            cout << "ColorRation[" << x*_nbCellsY+y << "]  =  "  << colorRation << endl;


            GroundColorMix(color,colorRation*150,0,1);
            glColor3d(color[0],color[1],color[2]);

            glVertex3d( (double)(center[0] - _cellSize[0]/2) , (double)(center[1] - _cellSize[1]/2), depth );
            glVertex3d( (double)(center[0] + _cellSize[0]/2) , (double)(center[1] - _cellSize[1]/2), depth );
            glVertex3d( (double)(center[0] + _cellSize[0]/2) , (double)(center[1] + _cellSize[1]/2), depth );
            glVertex3d( (double)(center[0] - _cellSize[0]/2) , (double)(center[1] + _cellSize[1]/2), depth );
        }
    }


    //    for(int i=0; i<nbCells; i++)
    //    {
    //        TwoDCell* cell = static_cast<TwoDCell*>(getCell(i));
    //        glColor4dv(colorvector);
    //        cell->draw();
    //    }

    glEnd();

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
}

PlanCell::PlanCell() :
        _Open(false),
        _Closed(false)
{

}

PlanCell::PlanCell(int i, Vector2d corner, PlanGrid* grid) :
        API::TwoDCell(i,corner,grid),
        _Open(false),
        _Closed(false),
        _CostIsComputed(false)
{
}

double PlanCell::getCost()
{
    Robot* rob = dynamic_cast<PlanGrid*>(_grid)->getRobot();

    double cost(0.0);
    const int nbRandShoot=3;

    for(int i=0;i<nbRandShoot;i++)
    {
        shared_ptr<Configuration> q = rob->shoot();
        q->getConfigStruct()[6] = this->getCenter()[0];
        q->getConfigStruct()[7] = this->getCenter()[1];
        //    q->print();
        cost += q->cost();
    }
    cost /= (double)nbRandShoot+ENV.getDouble(Env::Kdistance)+ENV.getDouble(Env::Kvisibility);
//    cout << "cost  = "  << cost << endl;
    return cost;
}
