#include "HRICS_Grid.h"
#include "HRICS_Cell.h"

using namespace std;

HriGrid::HriGrid()
{
}

HriGrid::HriGrid(vector<int> size)
{

}

HriGrid::HriGrid(double pace, vector<double> envSize) :
        Grid(pace,envSize)
{
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY*_nbCellsZ << endl;
}


/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
Cell* HriGrid::createNewCell(int index, int x, int y, int z )
{
    vector<int> pos;

    pos.push_back(x);
    pos.push_back(y);
    pos.push_back(z);

    //    cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;

    if (index == 0)
    {
        return new HriCell( 0, pos ,_originCorner , this );
    }
    return new HriCell( index, pos , computeCellCorner(x,y,z) , this );
}

/*!
 * \brief Compute Grid Cost
 */
void HriGrid::computeAllCellCost()
{
    int nbCells = this->getNumberOfCells();

    for(int i=0; i<nbCells; i++)
    {
        dynamic_cast<HriCell*>(getCell(i))->getHRICostSpace();
    }
}

/*!
 * \brief Reset Grid Cost
 */
void HriGrid::resetCellCost()
{
    int nbCells = this->getNumberOfCells();

    for(int i=0; i<nbCells; i++)
    {
        dynamic_cast<HriCell*>(getCell(i))->setBlankCost();
    }
}

/*!
 * \brief Draw Grid Cells
 */
void HriGrid::drawGrid()
{
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

    int nbCells = this->getNumberOfCells();

    for(int i=0; i<nbCells; i++)
    {
        HriCell* cell = static_cast<HriCell*>(getCell(i));
        double alpha = cell->getHRICostSpace();

        if(ENV.getInt(Env::hriCostType) == 0)
        {
            colorvector[1] = 0.5*(1-alpha)+0.5;
            colorvector[3] = 0.3*alpha+0.01;
        }
        if(ENV.getInt(Env::hriCostType) == 2 ||
           ENV.getInt(Env::hriCostType) == 3 )
        {
            colorvector[1] = 0.5*(1-alpha)+0.5;
            colorvector[3] = 0.01*(7-alpha)+0.01;
        }
        glColor4dv(colorvector);
//        g3d_set_color_mat(Any,colorvector);

        cell->drawCell();
    }

    glEnd();

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

//    glEnable(GL_LIGHTING);
//    glEnable(GL_LIGHT0);
}

