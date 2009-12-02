#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h" 
#include "Hri_planner-pkg.h"

#define MAX_ROW_INDEX 44 //10 centimeters
#define MAX_COL_INDEX 79
//#define MAX(a,b) (a<b)?b:a

static long gridWave[MAX_COL_INDEX][MAX_ROW_INDEX];
static long gridCost[MAX_COL_INDEX][MAX_ROW_INDEX];
static long gridObst[MAX_COL_INDEX][MAX_ROW_INDEX];
static int initializedGrids = 0;
static float envX1, envY1,envX2, envY2;
hri_bitmapset* WAVE_BTSET;



/////////////////////////////////////////////////////////////////////



/**************** Grid cleaners *************************/

static void cleanGrid(int maxX, int maxY)
{
  int i,j;
  for (i=0;i<maxX;i++)
    for (j=0;j<maxY;j++)
      gridCost[i][j]=-1;
}

static void cleanGridWave(int maxX, int maxY)
{
  int i,j;
  for (i=0;i<maxX;i++)
    for (j=0;j<maxY;j++)
		{
			gridWave[i][j]=-1;
		}
}

static void cleanAllGrid(int maxX, int maxY)
{
  cleanGrid(maxX, maxY);
  cleanGridWave(maxX, maxY);
}

/******************************************************/


static int inObs(int x, int y)
{
  int xaux,yaux;
  float xaux2,yaux2;
  float stepX = (envX2 - envX1)/MAX_COL_INDEX;
  float stepY = (envY2 - envY1)/MAX_ROW_INDEX;
  //printf("environment %f,%f  %f,%f ",envX1,envY1,envX2,envY2);
  //printf("steps %f  %f  \n", stepX,stepY);
  xaux2 = x * stepX + envX1;
  yaux2 = y * stepY + envY1;
  //printf("%i %i -> %f %f ",x,y,xaux2,yaux2);
  xaux = (xaux2 - WAVE_BTSET->realx)/ WAVE_BTSET->pace;
  yaux = (yaux2 - WAVE_BTSET->realy)/ WAVE_BTSET->pace;
  //printf("  -> %i %i \n",xaux,yaux);
  printf(" coords %i, %i Val %f  calculate %f\n",xaux, yaux, WAVE_BTSET->bitmap[BT_OBSTACLES]->data[xaux][yaux][0].val, WAVE_BTSET->bitmap[ BT_COMBINED ]->calculate_cell_value(WAVE_BTSET,xaux,yaux,0));
  if (WAVE_BTSET->bitmap[BT_OBSTACLES]->data[xaux][yaux][0].val == -2)
	{
		//printf("----- \n");
		return 1;
	}
  else
    if (WAVE_BTSET->bitmap[ BT_COMBINED ]->calculate_cell_value(WAVE_BTSET,xaux,yaux,0)==-2)
      return 1;
	
  return 0;
}


static int inObstacle(int x, int y)
{
	
  return gridObst[x][y];   
	
}



static void genGridObst(int maxCol, int maxRow)
{
	
  int i,j;
  
  WAVE_BTSET->bitmap[ BT_COMBINED ]->calculate_cell_value(WAVE_BTSET,1,1,0);
  for (i=0;i<maxCol;i++)
	{
		for (j=0;j<maxRow;j++)
		{
			if (inObs(i,j)) 
				gridObst[i][j] = 1;
			else
				gridObst[i][j] = 0;
		}
	}
	
	
}

static int getCellValue(int x, int y)
{
	
  if (x>=0 && y>=0 && x<MAX_COL_INDEX && y<MAX_ROW_INDEX) 
	{
		if (!inObstacle(x,y))
			return gridCost[x][y];
		else
			return -3;
	}
  return -2;
	
}

static int getCellCost(int x, int y)
{
	
  if (x>=0 && y>=0 && x<MAX_COL_INDEX && y<MAX_ROW_INDEX) 
	{
		return gridCost[x][y];
	}
  return -2;
	
}

static int getCellCostWave(int x, int y)
{
	
  if (x>=0 && y>=0 && x<MAX_COL_INDEX && y<MAX_ROW_INDEX) 
	{
		return gridWave[x][y];
	}
  return -2;
	
}



static int setNeighborsVal(int x, int y)
{
  int minc=-1;
  int cellValTmp;
  int band = 0;
	
  cellValTmp = getCellValue(x,y);
	
  if (cellValTmp == -2)
		return 0;
	
	
  if (cellValTmp == -3)
	{
		gridCost[x][y] = -3;
		return 0;
	}
	
  if (cellValTmp > -1)
    minc = cellValTmp;
  else
    band = 1;
	
  cellValTmp = getCellValue(x-1,y);
  if (minc>-1)
	{
		if (cellValTmp>-1 && cellValTmp+1 < minc)
			minc = cellValTmp + 1;
	}
  else
    if (cellValTmp>-1)
      minc = cellValTmp + 1;
	
  cellValTmp = getCellValue(x,y-1);
  if (minc>-1)
	{
		if (cellValTmp>-1 && cellValTmp+1 < minc)
			minc = cellValTmp + 1;
	}
  else
    if (cellValTmp>-1)
      minc = cellValTmp + 1;
	
	cellValTmp = getCellValue(x+1,y);
  if (minc>-1)
	{
		if (cellValTmp>-1 && cellValTmp+1 < minc)
			minc = cellValTmp + 1;
	}
  else
    if (cellValTmp>-1)
      minc = cellValTmp + 1;
	
	cellValTmp = getCellValue(x,y+1);
  if (minc>-1)
	{
		if (cellValTmp>-1 && cellValTmp+1 < minc)
			minc = cellValTmp + 1;
	}
  else
		if (cellValTmp>-1)
			minc = cellValTmp + 1;
	
  if (minc>-1)
	{
		gridCost[x][y] = minc;
		
		cellValTmp = getCellValue(x-1,y);
		if (cellValTmp==-1)
		{
			gridCost[x-1][y] = gridCost[x][y] + 1;
		}
		else
		{
			if (cellValTmp>-1)
				if (cellValTmp > gridCost[x][y] +1)
					gridCost[x-1][y] = gridCost[x][y] +1;
		}
		
		
		cellValTmp = getCellValue(x,y-1);
		if (cellValTmp==-1)
		{
			gridCost[x][y-1] = gridCost[x][y] + 1;
		}
		else
		{
			if (cellValTmp>-1)
				if (cellValTmp > gridCost[x][y] +1)
					gridCost[x][y-1] = gridCost[x][y] +1;
		}
		
		cellValTmp = getCellValue(x+1,y);
		if (cellValTmp==-1)
		{
			gridCost[x+1][y] = gridCost[x][y] + 1;
		}
		else
		{
			if (cellValTmp>-1)
				if (cellValTmp > gridCost[x][y] +1)
					gridCost[x+1][y] = gridCost[x][y] +1;
		}
		
		
		cellValTmp = getCellValue(x,y+1);
		if (cellValTmp==-1)
		{
			gridCost[x][y+1] = gridCost[x][y] + 1;
		}
		else
		{
			if (cellValTmp>-1)
				if (cellValTmp > gridCost[x][y] +1)
					gridCost[x][y+1] = gridCost[x][y] +1;
		}
		if (band)
			return 1;
	}
  return 0;
}




static void setSeed(int x, int y)
{
  int i=1, band=0;
	
  if (inObstacle(x,y))
	{
		
		while (!band)
		{
			
			if (!inObstacle(x+i,y-i))
	    {
	      band = 1;
	      gridCost[x+i][y-i] = 0;
	    }
			
			if (!inObstacle(x+i,y+i))
	    {
	      band = 1;
	      gridCost[x+i][y+i] = 0;
	    }
			
			if (!inObstacle(x-1,y+i))
	    {
	      band = 1;
	      gridCost[x-i][y+i] = 0;
	    }
			
  	  if (!inObstacle(x-i,y-i))
	    {
	      band = 1;
	      gridCost[x-i][y-i] = 0;
	    }
			
			i++;
		}
		printf("seed on an obstacle getting out in %i\n",i);
	}
  else
    gridCost[x][y] = 0;
  
}



static void generalWaveXpand(int x, int y)
{
  int totalcells = MAX_COL_INDEX*MAX_ROW_INDEX;
  int cellcount = 0;
  int i,j,changeflag = 1;
	
  setSeed(x,y);
	
  cellcount ++;
	
  // printf("cells on obstacles %i\n",cellcount);
  //while(cellcount<totalcells)
  while(changeflag)
	{
		changeflag = 0;
		for (i=0;i<MAX_COL_INDEX;i++)
		{
			for (j=0;j<MAX_ROW_INDEX;j++)
	    {
	      if (setNeighborsVal(i,j))
				{
					cellcount++;
					changeflag = 1;
				}
	      //printf("cell count  %i\n",cellcount);
			}
		}
	}
  
}


static void genWave(int x, int y)
{
  int maxi;
  gridCost[x][y] = 0;
  
  maxi = MAX(MAX_COL_INDEX+x,MAX_ROW_INDEX+y);
  maxi = MAX(maxi,MAX_ROW_INDEX+MAX_COL_INDEX);
	
  int i=1,j,k;
  while (i<=maxi)
	{
		//     printf("iteration %i\n",i);
		k=y;
		for (j=x-i;j<=x;j++)
		{
			// printf("row %i col %i\n",j,k);
			setNeighborsVal(j,k);
			//printf("--row %i col %i\n",x+(x-j),k);
			setNeighborsVal(x+(x-j),k);
			//printf("++row %i col %i\n",x,k-(y+k));
			setNeighborsVal(j,y+(y-k));
			//printf("==row %i col %i\n",x,k-(y+k));
			setNeighborsVal(x+(x-j),y+(y-k));
			
			k--;   
		}
		i++;
	}
	
}


/********************************* Interface Functions *******************************/


int igetCellCoord(float cLimInf, float cLimSup, int maxGridVal,  float cRealCoord)
{
  float dimAxe = (cLimSup - cLimInf)/maxGridVal;
  return (int)((cRealCoord-cLimInf)/dimAxe);
}

int InitWave(float x1, float y1, float x2, float y2, float waveX, float waveY)
{
  int x,y;
  cleanGrid(MAX_COL_INDEX,MAX_ROW_INDEX);
  envX1 = x1;
  envY1 = y1;
  envX2 = x2;
  envY2 = y2;
  //if(!hri_bt_is_active(BT_OBSTACLES,BTSET)) hri_bt_activate(BT_OBSTACLES,BTSET);
  //printf("Environment %f,%f %f,%f \n",x1,y1,x2,y2);
  x = igetCellCoord(x1,x2, MAX_COL_INDEX, waveX);
  y = igetCellCoord(y1,y2, MAX_ROW_INDEX, waveY);
  
  if (x>=0 && x<=MAX_COL_INDEX && y>=0 && y<=MAX_ROW_INDEX) 
	{
		
		//genWave(x,y);
		generalWaveXpand(x,y);
		PSP_init_grid = 1;
		//printf(" %f %f %f \n", WAVE_BTSET->realx, WAVE_BTSET->realy, WAVE_BTSET->pace);
		printf(" Expanded Wave\n");
		return 1;
	}
  printf("Wave Coordinates out of range\n");
  PSP_init_grid = 0;
  return 0;    
}

int InitWaveCells(float x1, float y1, float x2, float y2, float waveX, float waveY, hri_bitmapset* PSP_BTSET)
{
  cleanGridWave(MAX_COL_INDEX,MAX_ROW_INDEX);
  WAVE_BTSET = PSP_BTSET;
  envX1 = x1;
  envY1 = y1;
  envX2 = x2;
  envY2 = y2;
  genGridObst(MAX_COL_INDEX,MAX_ROW_INDEX);
  if (addWave(x1, y1, x2, y2, waveX, waveY))
	{  
		putGrid( MAX_COL_INDEX , MAX_ROW_INDEX );
		return 1;
	}
  return 0;
}

int InitAllWaveGrids(hri_bitmapset* PSP_BTSET)
{
  cleanAllGrid( MAX_COL_INDEX , MAX_ROW_INDEX );
  WAVE_BTSET = PSP_BTSET;
  genGridObst( MAX_COL_INDEX , MAX_ROW_INDEX );
}

int addWave(float x1, float y1, float x2, float y2, float waveX, float waveY)
{
	
  int waveres = InitWave(x1, y1, x2, y2,  waveX,  waveY);
	
  if (waveres)
	{
		pushGrid( MAX_COL_INDEX , MAX_ROW_INDEX );
		return 1;
	}
  return 0;    
}

int iget_wave_cost(float x, float y)
{
  int xaux,yaux;
  float costf;
  //int costi;
	
  if (!PSP_init_grid)
	{
		printf("WAVE EXPANSION: NON initialized grid\n");
		return -1;
	}
  //printf("Environment %f,%f %f,%f \n",envX1,envY1,envX2,envY2);
  xaux = igetCellCoord(envX1,envX2, MAX_COL_INDEX, x);
  yaux = igetCellCoord(envY1,envY2, MAX_ROW_INDEX, y);
  //printf("getting grid for coordinates %f, %f  to  %i,%i\n",x,y,xaux,yaux);
  costf = getCellCost(xaux,yaux);
  //costi = getCellCost(xaux,yaux);
  //printf("////...... COSTS Float %f, Int %i ......//// \n",costf,costi); 
  return  costf;
}

int iget_all_wave_cost(float x, float y)
{
	
  int xaux,yaux;
  int costf;
  //int costi;
	
  if (!PSP_init_grid)
	{
		printf("WAVE EXPANSION: NON initialized grid\n");
		return -1;
	}
  //printf("Environment %f,%f %f,%f \n",envX1,envY1,envX2,envY2);
  xaux = igetCellCoord(envX1,envX2, MAX_COL_INDEX, x);
  yaux = igetCellCoord(envY1,envY2, MAX_ROW_INDEX, y);
  //printf("getting grid for coordinates %f, %f  to  %i,%i\n",x,y,xaux,yaux);
  costf = getCellCostWave(xaux,yaux);
  //costi = getCellCost(xaux,yaux);
  //printf("////...... COSTS Float %f, Int %i ......//// \n",costf,costi); 
  return  costf;
	
}


long getMaxGridCost()
{
  int i,j;
  long max=0;
  if (!PSP_init_grid)
	{
		printf("WAVE EXPANSION: NON initialized grid\n");
		return -1;
	}
	
  for (i=0;i<MAX_COL_INDEX;i++)
	{
		for (j=0;j<MAX_ROW_INDEX;j++)
		{
			if (max<gridCost[i][j]) 
				max = gridCost[i][j];
		}
		
	}
  return max;
}

long getMaxWaveCost()
{
  int i,j;
  long max=0;
  if (!PSP_init_grid)
    {
      printf("WAVE EXPANSION: NON initialized grid\n");
      return -1;
    }

  for (i=0;i<MAX_COL_INDEX;i++)
    {
      for (j=0;j<MAX_ROW_INDEX;j++)
	{
	  if (max<gridCost[i][j]) 
	    max = gridWave[i][j];
	}

    }
  return max;
}

long wv_getMaxWaveCostOf(double x1, double y1, double x2, double y2)
{
	int i,j,xaux,yaux,xaux2,yaux2;
	long max=0;
	if (!PSP_init_grid)
	{
		printf("WAVE EXPANSION: NON initialized grid\n");
		return -1;
	}
	xaux = igetCellCoord(envX1,envX2, MAX_COL_INDEX, x1);
	yaux = igetCellCoord(envY1,envY2, MAX_ROW_INDEX, y1);
	xaux2 = igetCellCoord(envX1,envX2, MAX_COL_INDEX, x2);
	yaux2 = igetCellCoord(envY1,envY2, MAX_ROW_INDEX, y2);
	
	for (i=MIN(x1,x2);i<MAX(x1,x2);i++)
	{
		for (j=MIN(y1,y2);j<MAX(y1,y2);j++)
		{
			if (max<gridCost[i][j]) 
				max = gridWave[i][j];
		}
		
	}
  return max;
}

void pushGrid(int maxX, int maxY)
{
	
  int i,j;
  //if (initializedGrids)
	for (i=0;i<maxX;i++)
		for (j=0;j<maxY;j++)
		{
			if (!inObstacle(i,j))
				gridWave[i][j]+=gridCost[i][j];
			else
				gridWave[i][j]=gridCost[i][j];
		}
}


void putGrid(int maxX, int maxY)
{
	
  int i,j;
  //if (initializedGrids)
	for (i=0;i<maxX;i++)
		for (j=0;j<maxY;j++)
		{
	    gridWave[i][j]=gridCost[i][j];
		}
}

/* **************************************************************
 
 
 Print grid functions
 
 
 ****************************************************************/



static void printGrid(int x, int y)
{
  int i,j;
  for (i=0;i<x;i++)
	{
		for (j=0;j<y;j++)
		{
			if (gridCost[i][j]>-1) 
				printf(" : ");
			else
				printf(" . ");
		}
		printf("\n");
	}
}

void printGridVals()
{
  int i,j;
  FILE * gdf;
  gdf = fopen("grid.dat","w");
  if (gdf  == NULL)
	{
		printf("WAVE EXPANSION: Can not open file grid.dat.\n");
		return;
	}
	
  for (i=0;i<MAX_COL_INDEX;i++)
	{
		for (j=0;j<MAX_ROW_INDEX;j++)
		{
			//if (gridCost[i][j]>=-1) 
	    fprintf(gdf,"%i %i %i \n",i,j,gridCost[i][j]);
		}
		fprintf(gdf,"\n");
	}
  fclose(gdf);
}

void printWaveVals()
{
  int i,j;
  FILE * gdf;
  gdf = fopen("gridWave.dat","w");
  if (gdf  == NULL)
	{
		printf("WAVE EXPANSION: Can not open file grid.dat.\n");
		return;
	}
	
  for (i=0;i<MAX_COL_INDEX;i++)
	{
		for (j=0;j<MAX_ROW_INDEX;j++)
		{
			//if (gridCost[i][j]>=-1) 
	    fprintf(gdf,"%i %i %i \n",i,j,gridWave[i][j]);
		}
		fprintf(gdf,"\n");
	}
  fclose(gdf);
}

void printObstacles()
{
  int i,j,k;
  FILE * gdf;
  gdf = fopen("obstacles.dat","w");
  if (gdf  == NULL)
	{
		printf("WAVE EXPANSION: Can not open file obstacles.\n");
		return;
	}
	
  printf("---- nx %i ny %i \n", WAVE_BTSET->bitmap[BT_OBSTACLES]->nx,WAVE_BTSET->bitmap[BT_OBSTACLES]->ny);
	
  for (i=0;i<WAVE_BTSET->bitmap[BT_OBSTACLES]->nx;i++)
	{
		for (j=0;j<WAVE_BTSET->bitmap[BT_OBSTACLES]->ny;j++)
		{
			//if (gridCost[i][j]>=-1) 
			for (k=0;k<WAVE_BTSET->bitmap[BT_OBSTACLES]->nz;k++)
				fprintf(gdf,"%i %i %f \n",i,j,WAVE_BTSET->bitmap[BT_OBSTACLES]->data[i][j][k].val);
		}
		fprintf(gdf,"\n");
	}
  fclose(gdf);
}

void printBitmap()
{
  int i,j,k;
  FILE * gdf;
  gdf = fopen("bitmap.dat","w");
  if (gdf  == NULL)
	{
		printf("WAVE EXPANSION: Can not open file bitmap.dat.\n");
		return;
	}
	
  for (i=0;i<WAVE_BTSET->bitmap[BT_OBSTACLES]->nx;i++)
	{
		for (j=0;j<WAVE_BTSET->bitmap[BT_OBSTACLES]->ny;j++)
		{
			//if (gridCost[i][j]>=-1) 
			for (k=0;k<WAVE_BTSET->bitmap[BT_OBSTACLES]->nz;k++)
				fprintf(gdf,"%i %i %f \n",i,j,WAVE_BTSET->bitmap[BT_OBSTACLES]->data[i][j][k].val);
		}
		fprintf(gdf,"\n");
	}
  fclose(gdf);
}

void printCombinedBitmap()
{
  int i,j,k;
  FILE * gdf;
  gdf = fopen("combBitmap.dat","w");
  if (gdf  == NULL)
	{
		printf("WAVE EXPANSION: Can not open file combBitmap.dat.\n");
		return;
	}
	
  for (i=0;i<WAVE_BTSET->bitmap[BT_OBSTACLES]->nx;i++)
	{
		for (j=0;j<WAVE_BTSET->bitmap[BT_OBSTACLES]->ny;j++)
		{
			//if (gridCost[i][j]>=-1) 
			for (k=0;k<WAVE_BTSET->bitmap[BT_OBSTACLES]->nz;k++)
	    {
				fprintf(gdf,"%i %i %f \n",i,j,WAVE_BTSET->bitmap[BT_COMBINED]->calculate_cell_value(WAVE_BTSET,i,j,k));
				//printf("z = %i\n",k);
	    }
		}
		fprintf(gdf,"\n");
	}
  fclose(gdf);
}

void printGridObst()
{
  int i,j;
  FILE * gdf;
  gdf = fopen("gridobst.dat","w");
  if (gdf == NULL)
	{
		printf("WAVE EXPANSION: Can not open file gridobst.dat.\n");
		return;
	}
	
  for (i=0;i<MAX_COL_INDEX;i++)
	{
		for (j=0;j<MAX_ROW_INDEX;j++)
		{
			if (!inObstacle(i,j)) 
				fprintf(gdf,"%i %i %i \n",i,j,0);
			else
				fprintf(gdf,"%i %i %i \n",i,j,1);
		}
		fprintf(gdf,"\n");
	}
  fclose(gdf);
}
