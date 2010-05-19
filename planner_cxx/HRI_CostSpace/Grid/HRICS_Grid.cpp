#include "HRICS_Grid.h"
#include "HRICS_Cell.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

Grid::Grid()
{
}

Grid::Grid(vector<int> size)
{

}

Grid::Grid(double pace, vector<double> envSize) :
        API::ThreeDGrid(pace,envSize)
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
API::ThreeDCell* Grid::createNewCell(unsigned int index, unsigned int x, unsigned int y, unsigned int z )
{
    Vector3i pos;

    pos[0] = x;
    pos[1] = y;
    pos[2] = z;

    //    cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;

    if (index == 0)
    {
        return new Cell( 0, pos ,_originCorner , this );
    }
    return new Cell( index, pos , computeCellCorner(x,y,z) , this );
}

/*!
 * \brief Compute Grid Cost
 */
void Grid::computeAllCellCost()
{
    int nbCells = this->getNumberOfCells();

    shared_ptr<Configuration> robotConf = _Robot->getCurrentPos();
    for(int i=0; i<nbCells; i++)
    {
//        dynamic_cast<Cell*>( BaseGrid::getCell(i) )->getHRICostSpace();
        dynamic_cast<Cell*>( BaseGrid::getCell(i) )->getCost();
    }
    _Robot->setAndUpdate(*robotConf);
    API_activeGrid = this;
}

/*!
 * \brief Reset Grid Cost
 */
void Grid::resetCellCost()
{
    int nbCells = this->getNumberOfCells();

    for(int i=0; i<nbCells; i++)
    {
        dynamic_cast<Cell*>( BaseGrid::getCell(i) )->setBlankCost();
    }
}

/*!
 * \brief Draw Grid Cells
 */
void Grid::drawSpheres()
{
    double colorvector[4];
	
    colorvector[0] = 1.0;       //red
    colorvector[1] = 0.5;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 0.05;       //transparency
	
	int nbCells = this->getNumberOfCells();
	
	for(int i=0; i<nbCells; i++)
    {
        Cell* cell = dynamic_cast<Cell*>( BaseGrid::getCell(i) );
        
		if ((!cell->getIsCostComputed())) 
		{
			cell->createDisplaylist();
		}
	}
	
    //cout << "Drawing HRICS::Grid::draw()"  << endl;
	
    for(int i=0; i<nbCells; i++)
    {
        Cell* cell = dynamic_cast<Cell*>( BaseGrid::getCell(i) );
        
		double alpha = cell->getCost();
		
		if(ENV.getInt(Env::hriCostType) == 0)
        {
			if(alpha < 0.1 )
				continue;
			
			// Alpha goes from 0 to 1 + Epsilon
			//GroundColorMix(colorvector,360-360*alpha+100,0,1);
			GroundColorMixGreenToRed(colorvector,alpha);
            //colorvector[1] = 0.5*(1-ENV.getDouble(Env::colorThreshold2)*alpha)+0.5;
            //colorvector[3] = 0.3*ENV.getDouble(Env::colorThreshold1)*alpha; //+0.01;
        }
		
        if( ENV.getInt(Env::hriCostType) == 1 )
        {
			if(alpha > 0.05)
				continue;
			
			GroundColorMixGreenToRed(colorvector,alpha);
			
            //colorvector[1] = 0.5*(1-10*ENV.getDouble(Env::colorThreshold2)*alpha)+0.5;
            //colorvector[3] = 0.1*(0.7-ENV.getDouble(Env::colorThreshold1)*alpha)+0.01;
        }
        glColor4dv(colorvector);
        //        g3d_set_color_mat(Any,colorvector);
		
        //cell->draw();
		glCallList(cell->getDisplayList());
    }
}

/*!
 * \brief Draw Grid Cells
 */
void Grid::draw()
{
	
	return drawSpheres();
	
    double colorvector[4];

    colorvector[0] = 0.2;       //red
    colorvector[1] = 1.0;       //green
    colorvector[2] = 0.2;       //blue
    colorvector[3] = 0.05;       //transparency

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

 //   glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);
	
	int nbCells = this->getNumberOfCells();

    //cout << "Drawing HRICS::Grid::draw()"  << endl;

    for(int i=0; i<nbCells; i++)
    {
        Cell* cell = dynamic_cast<Cell*>( BaseGrid::getCell(i) );
        
		double alpha = cell->getCost();
		
		GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*alpha);
		
		if(ENV.getInt(Env::hriCostType) == 0)
        {
//			if(alpha < ENV.getDouble(Env::colorThreshold1))
//				continue;
			
			// Alpha goes from 0 to 1 + Epsilon
			//GroundColorMix(colorvector,360-360*alpha+100,0,1);
            //colorvector[1] = 0.5*(1-ENV.getDouble(Env::colorThreshold2)*alpha)+0.5;
            colorvector[3] = 0.20*ENV.getDouble(Env::colorThreshold2)*alpha; //+0.01;
        }
		
        if(ENV.getInt(Env::hriCostType) == 1 )
		{
			colorvector[3] = ENV.getDouble(Env::colorThreshold2)*1/alpha; 
		}
			
        if ( ENV.getInt(Env::hriCostType) == 2 )
        {
			alpha /= (ENV.getDouble(Env::Kdistance)+ENV.getDouble(Env::Kvisibility));
			
			//GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*alpha);
			
            //colorvector[1] = 0.5*(1-10*ENV.getDouble(Env::colorThreshold2)*alpha)+0.5;
            //colorvector[3] = 0.20*(1-ENV.getDouble(Env::colorThreshold2)*alpha);
			colorvector[3] = 0.20*ENV.getDouble(Env::colorThreshold2)*alpha; 
        }
        glColor4dv(colorvector);
        //        g3d_set_color_mat(Any,colorvector);

        cell->draw();
    }

    glEnd();

//    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
}


bool Grid::isVirtualObjectPathValid(Cell* fromCell,Cell* toCell)
{
//    shared_ptr<Configuration> configFrom(new Configuration(_Robot));
//
//    vector<double> cellCenter = fromCell->getCenter();
//
//    configFrom->print();

//    configFrom->getConfigStruct()[ENV.getInt(Env::akinJntId)] = cellCenter[0];
//    configFrom->getConfigStruct()[7] = cellCenter[1];
//    configFrom->getConfigStruct()[8] = cellCenter[2];
//    configFrom->getConfigStruct()[9] =    0;
//    configFrom->getConfigStruct()[10] =   0;
//    configFrom->getConfigStruct()[11] =   0;

    return true;
}

