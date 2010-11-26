extern HRI_KNOWLEDGE * hri_create_empty_agent_knowledge(HRI_AGENT * hri_agent);
extern HRI_ENTITIES * hri_create_entities();
extern int hri_refine_entity_types(HRI_ENTITIES * entities, HRI_AGENTS * agents);
extern HRI_REACHABILITY hri_is_reachable(HRI_ENTITY * object, HRI_AGENT *agent);
extern HRI_PLACEMENT_RELATION hri_placement_relation(p3d_rob *sourceObj, p3d_rob *targetObj);
extern int hri_is_on(p3d_vector3 topObjC, p3d_BB *topObjBB, p3d_BB *bottomObjBB);
extern int hri_is_in(p3d_vector3 insideObjC, p3d_BB *outsideObjBB);
extern int hri_is_nexto(p3d_vector3 sourceC, p3d_BB *sourceBB, p3d_vector3 targetC, p3d_BB *targetBB);
extern HRI_SPATIAL_RELATION hri_spatial_relation(p3d_rob * object, p3d_rob * robot);
