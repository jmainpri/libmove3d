#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

HRI_COLOR_MASK *HRI_GLOBAL_COLOR_MASK;

void g3d_hri_display_visible_objects(HRI_AGENT *agent)
{
  p3d_env *env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int i;
  
  for(i=0; i<agent->perspective->currently_sees.vis_nb; i++) {
    if(agent->perspective->currently_sees.vis[i] == HRI_VISIBLE) {
      p3d_set_robot_display_mode(env->robot[i], P3D_ROB_RED_DISPLAY);
    }
    else {
      p3d_set_robot_display_mode(env->robot[i], P3D_ROB_DEFAULT_DISPLAY);
    }
  }
}