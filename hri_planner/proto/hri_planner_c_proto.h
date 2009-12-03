extern HRI_AGENTS * hri_create_agents(void);
extern HRI_AGENT * hri_create_agent(p3d_rob * robot);
extern HRI_MANIP * hri_create_empty_agent_manip();
extern HRI_MANIP * hri_create_agent_manip(HRI_AGENT * agent);
extern int hri_fill_all_agents_default_tasks(HRI_AGENTS * agents);
extern int hri_create_fill_agent_default_manip_tasks(GIK_TASK ** tasklist, int * tasklist_no, HRI_AGENT_TYPE type);
extern int hri_create_assign_default_manipulation(HRI_AGENTS * agents);
extern int hri_agent_single_task_manip_move(HRI_AGENT * agent, HRI_GIK_TASK_TYPE type, p3d_vector3 * goalCoord, configPt *q);
