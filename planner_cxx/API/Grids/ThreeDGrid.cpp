#include "ThreeDGrid.h"
#include "Graphic-pkg.h"

using namespace std;
using namespace API;

#include <iostream>


/*!
 * \brief Constructor
 *
 * \param X number of cells
 * \param Y number of cells
 * \param Z number of cells
 */
ThreeDGrid::ThreeDGrid()
{

}

/*!
 * \brief Destructor
 */
ThreeDGrid::~ThreeDGrid()
{
    for(unsigned int i=0;i<_cells.size();i++)
    {
        delete _cells.at(i);
    }
}

/*!
 * \brief Initializes the grid with a pace
 *
 * \param vector int size (number of cells in X, Y, Z)
 * \param vector envSize XMin Xmax YMin YMax ZMin ZMax
 */
ThreeDGrid::ThreeDGrid( Vector3i size, vector<double> envSize )

{
    _nbCellsX = size[0];
    _nbCellsY = size[1];
    _nbCellsZ = size[2];

    _cellSize[0] = (envSize.at(1) - envSize.at(0)) / _nbCellsX ;
    _cellSize[1] = (envSize.at(3) - envSize.at(2)) / _nbCellsY ;
    _cellSize[2] = (envSize.at(5) - envSize.at(4)) / _nbCellsZ ;

    _originCorner[0] = envSize.at(0);
    _originCorner[0] = envSize.at(2);
    _originCorner[0] = envSize.at(4);

//    cout << "_originCorner[0] = " << _originCorner.at(0) <<  endl;
//    cout << "_originCorner[1] = " << _originCorner.at(1) <<  endl;
//    cout << "_originCorner[2] = " << _originCorner.at(2) <<  endl;
}


/*!
 * \brief Initializes the grid with a certain pace
 *
 * \param double pace : sizes of the squared cells IMPORTANT Cells are squared
 * \param vector envSize XMin Xmax YMin YMax ZMin ZMax
 */
ThreeDGrid::ThreeDGrid( double samplingRate, vector<double> envSize )
{
    for(unsigned int i= 0; i< envSize.size() ; i++)
    {
        cout << envSize.at(i) << " ";
    }
    cout << endl;

    if(((int)samplingRate) != 0 )
    {
        if( ( ((int) (envSize.at(1) - envSize.at(0))) % (int)samplingRate ) != 0 )
        {
            cout << "ThreeDGrid Warning : not good X disctretization " << endl;
        }

        if( ( ((int) (envSize.at(3) - envSize.at(2))) % (int)samplingRate ) != 0 )
        {
            cout << "ThreeDGrid Warning : not good Y disctretization " << endl;
        }

        if( ( ((int) (envSize.at(5) - envSize.at(4))) % (int)samplingRate ) != 0 )
        {
            cout << "ThreeDGrid Warning : not good Z disctretization " << endl;
        }
    }

//    _cellSize.push_back( (envSize.at(1) - envSize.at(0))/pace );
//    _cellSize.push_back( (envSize.at(3) - envSize.at(2))/pace );
//    _cellSize.push_back( (envSize.at(5) - envSize.at(4))/pace );

    _cellSize[0] =  samplingRate ;
    _cellSize[1] =  samplingRate ;
    _cellSize[2] =  samplingRate ;

    _nbCellsX =  (envSize.at(1) - envSize.at(0)) / samplingRate ;
    _nbCellsY =  (envSize.at(3) - envSize.at(2)) / samplingRate ;
    _nbCellsZ =  (envSize.at(5) - envSize.at(4)) / samplingRate ;

    cout << " _nbCellsX = " << _nbCellsX << endl;
    cout << " _nbCellsY = " << _nbCellsY << endl;
    cout << " _nbCellsZ = " << _nbCellsZ << endl;

    _originCorner[0] = envSize.at(0);
    _originCorner[1] = envSize.at(2);
    _originCorner[2] = envSize.at(4);

    cout << "_originCorner[0] = " << _originCorner[0] <<  endl;
    cout << "_originCorner[1] = " << _originCorner[1] <<  endl;
    cout << "_originCorner[2] = " << _originCorner[2] <<  endl;

}

/*!
 * \brief Creates All Cells
 *
 * \param vector envSize XMin Xmax YMin YMax ZMin ZMax
 */
void ThreeDGrid::createAllCells()
{

    int _nbCells = _nbCellsX * _nbCellsY * _nbCellsZ;

    _cells.resize(_nbCells);

    int x=0;
    int y=0;
    int z=0;

    for(int i = 0; i < _nbCells; i++)
    {
//        cout << "("<< x << "," << y << "," << z << ")" << endl;

        ThreeDCell* ptrCell = createNewCell(i,x,y,z);
        _cells[i] = ptrCell;

        x++;
        if( x >= _nbCellsX )
        {
            y++;
            x=0;
            if( y >= _nbCellsY )
            {
                z++;
                y=0;
                if( z > _nbCellsZ )
                {
                    cout << "ThreeDGrid : Error Size of ThreeDGrid " << endl;
                    return;
                }
            }
        }
    }
//    cout << "Finished"<< endl;
}

/*!
 * \brief Retruns the Cell at (x,y,z)
 *
 * \param integers x, y, z
 */
ThreeDCell* ThreeDGrid::getCell(int x, int y, int z)
{
    if(x<0 || x >= _nbCellsX)
    {
        cout << "ThreeDGrid Error : out of bands"<< endl;
    }
    if(y<0 || y >= _nbCellsY)
    {
        cout << "ThreeDGrid Error : out of bands"<< endl;
    }
    if(z<0 || z >= _nbCellsZ)
    {
        cout << "ThreeDGrid Error : out of bands"<< endl;
    }

    return dynamic_cast<ThreeDCell*>(_cells[ x + y*_nbCellsX + z*_nbCellsX*_nbCellsY ]);
}

/*!
 * \brief Get Cell
 *
 * \param index
 */
ThreeDCell* ThreeDGrid::getCell(Vector3i cell)
{
    return getCell(cell[0],cell[1],cell[2]);
}

/*!
 * \brief Get Cell in 3D ThreeDGrid
 *
 * \param index
 */
ThreeDCell* ThreeDGrid::getCell(Vector3d point)
{
    int x = (int)floor((abs(point[0]-_originCorner[0]))/_cellSize[0]);
    int y = (int)floor((abs(point[1]-_originCorner[1]))/_cellSize[1]);
    int z = (int)floor((abs(point[2]-_originCorner[2]))/_cellSize[2]);

//    cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;

    if( x>=_nbCellsX ||  y>=_nbCellsY || z>=_nbCellsZ || x<0 || y<0 || z<0 )
    {
       cout << "ThreeDGrid:: OutBands " << endl;
       return 0x0;
     }

    return getCell(x,y,z);
}

/*!
 * \brief Get Cell in 3D ThreeDGrid
 *
 * \param index
 */
ThreeDCell* ThreeDGrid::getCell(double* pos)
{
  int x = (int)((pos[0]-_originCorner[0])/_cellSize[0]);
  int y = (int)((pos[1]-_originCorner[1])/_cellSize[1]);
  int z = (int)((pos[2]-_originCorner[2])/_cellSize[2]);
  
  //    cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;
  
  if( x>=_nbCellsX ||  y>=_nbCellsY || z>=_nbCellsZ || x<0 || y<0 || z<0 )
  {
    cout << "ThreeDGrid:: OutBands " << endl;
    return 0x0;
  }
  
  return getCell(x,y,z);
}

/*!
 * \brief Get place in grid
 *
 * \param index
 */
Vector3i ThreeDGrid::getCellCoord(ThreeDCell* ptrCell)
{
   Vector3i coord;

   int i = ptrCell->getIndex();

   coord[0] = (i/1) % 3 - 1 ; // x
   coord[1] = (i/3) % 3 - 1 ; // y
   coord[2] = (i/9) % 3 - 1 ; // z

   return coord;
}


/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
ThreeDCell* ThreeDGrid::createNewCell(int index, int x, int y, int z )
{
    if (index == 0)
    {
        return new ThreeDCell( 0, _originCorner , this );
    }
    ThreeDCell* newCell = new ThreeDCell( index, computeCellCorner(x,y,z) , this );
    Vector3d corner = newCell->getCorner();
//    cout << " = (" << corner[0] <<"," << corner[1] << "," << corner[2] << ")" << endl;
    return newCell;
}

/*!
 * \brief Computes the corner of a cell
 *
 * \param integer index
 */
Vector3d ThreeDGrid::computeCellCorner(int x, int y, int z)
{
    Vector3d corner;

    corner[0] = _originCorner[0] + x*_cellSize[0];
    corner[1] = _originCorner[1] + y*_cellSize[1];
    corner[2] = _originCorner[2] + z*_cellSize[2];

//    cout << " = (" << x <<"," << y << "," << z << ")" << endl;
//    cout << " = (" << corner[0] <<"," << corner[1] << "," << corner[2] << ")" << endl;

    return corner;
}


/*!
 * \brief Get Number Of Cells
 */
int ThreeDGrid::getNumberOfCells()
{
    return _cells.size();
}


/*!
 * \brief Get Neighboor Cell
 */
ThreeDCell* ThreeDGrid::getNeighbour( const Vector3i& pos, int i)
{
    if( i<0 || i>26 )
    {
        return 0x0;
    }
    else
    {
        if(i>=13) i++;

        int dx =  (i/1) % 3 - 1 ;
        int dy =  (i/3) % 3 - 1 ;
        int dz =  (i/9) % 3 - 1 ;

//        cout << "( "<<dx<<" , "<<dy<<" , "<<dz<<" ) "<< endl;

        int x = pos[0] + dx ;
        int y = pos[1] + dy ;
        int z = pos[2] + dz ;

        if( x>=_nbCellsX ||  y>=_nbCellsY || z>=_nbCellsZ || x<0 || y<0 || z<0 )
        {
//            cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;
//            cout << "OutBands" << endl;
            return 0x0;
        }
        else
        {
//            cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;
            return getCell(x,y,z);
        }
    }
}

/**
 * Retrive the X Y Z coordinate of the cell from its index
 */
Vector3d ThreeDGrid::getCoordinates(ThreeDCell* cell)
{
  Vector3d coordinates;
  int index = cell->getIndex();
  int sizeXY = _nbCellsX * _nbCellsY;
  coordinates[2] = floor(index / sizeXY);
  coordinates[1] = floor((index - coordinates[2]*sizeXY) / _nbCellsX);
  coordinates[0] = floor(index - coordinates[2]*sizeXY - coordinates[1] * _nbCellsX);
  return coordinates;
}

void ThreeDGrid::draw()
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
        ThreeDCell* cell = dynamic_cast<ThreeDCell*>( BaseGrid::getCell(i) );
        glColor4dv(colorvector);
        cell->draw();
    }

    glEnd();

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
}