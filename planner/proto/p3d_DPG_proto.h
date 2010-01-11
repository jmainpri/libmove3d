#ifndef __CEXTRACT__
#ifdef DPG

extern p3d_dpgGrid * p3d_allocDPGGrid(void);
extern void p3d_destroyDPGGrid(p3d_dpgGrid ** grid);
extern void p3d_initDPGGrid(p3d_env * env, p3d_dpgGrid * grid);
extern p3d_dpgCell * p3d_allocDPGCell(void);
extern void p3d_destroyDPGCell(p3d_dpgCell ** cell);
extern void p3d_initDPGCell(p3d_dpgCell * cell);
extern void p3d_initStaticGrid(p3d_env * env, p3d_dpgGrid * grid);
extern void buildEnvEdges(p3d_env * env);

#endif
#endif /* __CEXTRACT__ */
