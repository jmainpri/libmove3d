extern HRI_KNOWLEDGE * hri_create_empty_agent_knowledge(HRI_AGENT * hri_agent);
extern HRI_ENTITIES * hri_create_entities();
extern int hri_refine_entity_types(HRI_ENTITIES * entities, HRI_AGENTS * agents);
extern HRI_REACHABILITY hri_is_reachable(HRI_ENTITY * object, HRI_AGENT *agent);