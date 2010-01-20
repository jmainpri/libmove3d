extern void p3d_SetIsHriFuncSpace(int IsHriFuncSpace);
extern int p3d_GetIsHriFuncSpace();
extern void p3d_SetHriZoneSize(double size);
extern void p3d_AddHriZone();
extern double p3d_GetHriZoneSize();
extern double* p3d_GetVectJim();
extern void p3d_DeactivateAllButHri(int disp);
extern void p3d_ActivateAll(int disp);
extern double p3d_GetHriDistCost(p3d_rob* robotPt, int disp);
extern void p3d_SaveGPlotCostTraj(void);
extern void p3d_SaveGPlotCostNodeChro(void);
extern double p3d_GetLpHriDistCost(p3d_rob* robotPt, p3d_localpath * lp);
// void p3d_SetCostToTab(p3d_rob *robotPt,  conf_cost * tab, int nbPart);
// void p3d_WriteConfCostToCSV(FILE *fd, conf_cost *tab, int size);
// void p3d_WriteConfCostToOBPlane(FILE *fd, conf_cost *tab, int size);
