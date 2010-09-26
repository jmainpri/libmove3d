#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

void g3d_hri_display_visible_objects(HRI_AGENT *agent)
{
  p3d_env *env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int i;
  
  agent->perspective->enable_vision_draw = TRUE;
  
  for(i=0; i<agent->perspective->currently_sees.vis_nb; i++) {
    if(agent->perspective->currently_sees.vis[i] == HRI_VISIBLE) {
      if(agent->perspective->currently_sees.vispl[i] == HRI_FOA) {
        p3d_set_robot_display_mode(env->robot[i], P3D_ROB_GREEN_DISPLAY);
      }
      //else {
      //  p3d_set_robot_display_mode(env->robot[i], P3D_ROB_BLUE_DISPLAY);
      //}
    }
    else {
      if (agent->perspective->currently_sees.vispl[i] == HRI_FOA || agent->perspective->currently_sees.vispl[i] == HRI_FOV) {
        p3d_set_robot_display_mode(env->robot[i], P3D_ROB_RED_DISPLAY);
      }
      else {
        p3d_set_robot_display_mode(env->robot[i], P3D_ROB_DEFAULT_DISPLAY);
      }
    }
  }
}


void g3d_hri_display_all_agents_sees(HRI_AGENTS *agents)
{
  int i;
  
  if(agents != NULL){
    for (i=0; i<agents->all_agents_no; i++) {
      if(agents->all_agents[i]->perspective->enable_visible_objects_draw)  
        g3d_hri_display_visible_objects(agents->all_agents[i]);
    }
  }
}



