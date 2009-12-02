#include "gridtograph.h"
#include "celltonode.h"
#include "../cell.h"

#include <vector>
#include "P3d-pkg.h"
#include <iostream>

using namespace std;
using namespace tr1;

GridToGraph::GridToGraph() : Grid()
{

}

GridToGraph::GridToGraph( vector<int> size ) : Grid()
{
    _nbCellsX = size[0];
    _nbCellsY = size[1];
    _nbCellsZ = size[2];

    _cellSize.push_back( (XYZ_ENV->box.x1 - XYZ_ENV->box.x2) / _nbCellsX );
    _cellSize.push_back( (XYZ_ENV->box.y1 - XYZ_ENV->box.y2) / _nbCellsY );
    _cellSize.push_back( (XYZ_ENV->box.z1 - XYZ_ENV->box.z2) / _nbCellsZ );

    _originCorner.push_back(XYZ_ENV->box.x2);
    _originCorner.push_back(XYZ_ENV->box.y2);
    _originCorner.push_back(XYZ_ENV->box.z2);

    _Robot = new Robot(XYZ_ROBOT);
    _Graph = new Graph(_Robot,XYZ_GRAPH);
    XYZ_GRAPH = _Graph->getGraphStruct();

}

GridToGraph::GridToGraph( double pace, vector<double> envSize ) :
        Grid( pace, envSize )
{
    _Robot = new Robot(XYZ_ROBOT);
    _Graph = new Graph(_Robot,XYZ_GRAPH);

}

/*!
 * \brief 3D robot from grid to graph
 */
void GridToGraph::putGridInGraph()
{
    createAllCells();

//    return;

    int edges=1;
    int newPath=1;
    int newNodes=1;

    for(int i=0;i<_nbCellsX;i++)
    {
        for(int j=0;j<_nbCellsY;j++)
        {
            for(int k=0;k<_nbCellsZ;k++)
            {
                CellToNode* currentCell = dynamic_cast<CellToNode*>( getCell(i,j,k) );

                vector<double> center = currentCell->getCenter();

                shared_ptr<Configuration> conf(new Configuration(_Robot));
                conf = _Robot->getCurrentPos();

                conf->getConfigStruct()[6] = center[0];
                conf->getConfigStruct()[7] = center[1];
                conf->getConfigStruct()[8] = center[2];
                conf->getConfigStruct()[9] =    0;
                conf->getConfigStruct()[10] =   0;
                conf->getConfigStruct()[11] =   0;
//                conf->print();

                vector<double> corner = currentCell->getCorner();

                                //  cout << "--------------------------------------------" << endl;
//                cout << newNodes << " = (" << i <<"," << j << "," << k << ")" << endl;
//                cout << newNodes << " = (" << corner[0] <<"," << corner[1] << "," << corner[2] << ")" << endl;
//                cout << newNodes << " = (" << center[0] <<"," << center[1] << "," << center[2] << ")" << endl;

                Node* newNode = new Node(_Graph,conf);

                if( (i==0) && (j ==0) && (k == 0) )
                {
                    _Graph->insertNode(newNode);
                    _Graph->linkToAllNodes(newNode);
                    currentCell->setNode(newNode);
                    continue;
                }


                newNodes++;
                _Graph->insertNode(newNode);
                currentCell->setNode(newNode);

                vector<int> pos(3);

                pos[0] = i;
                pos[1] = j;
                pos[2] = k;

                for(int l=0;l<26;l++)
                {
                    CellToNode* ptrCell = dynamic_cast<CellToNode*>(getNeighbour(pos,l));

                    if( ptrCell != 0 )
                    {
                        if(ptrCell->cellHasNode())
                        {
                            newPath++;
                            LocalPath path(ptrCell->getNode()->getConfiguration(),conf);

                            if(path.getValid())
                            {
//                                double Length = ptrCell->getNode()->getConfiguration()->dist(*conf);
                                _Graph->insertConfigurationAsNode(conf,ptrCell->getNode(),path.length());
                                edges++;
//                                cout << "Cell Index  = " <<  currentCell->getIndex() << " " << ptrCell->getIndex()  << endl;
                            }
                        }
                    }
                }
            }
        }
    }

    cout << newNodes << " Nodes" << endl;
    cout << edges << " Edges in Graph" << endl;
    cout << newPath << " NewPaths in Graph" << endl;
}

/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
Cell* GridToGraph::createNewCell(int index, int x, int y, int z )
{
    if (index == 0)
    {
        return new CellToNode( 0, _originCorner , this );
    }
    CellToNode* newCell = new CellToNode( index, computeCellCorner(x,y,z) , this );
    vector<double> corner = newCell->getCorner();
//    cout << " = (" << corner[0] <<"," << corner[1] << "," << corner[2] << ") newCell" << endl;
    return newCell;
}
