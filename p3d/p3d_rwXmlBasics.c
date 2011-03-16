/*
 *  p3d_rwXmlBasics.c
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 14/03/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#include "P3d-pkg.h"
#include "Localpath-pkg.h"

//! Write a matrix to a file
//! 
void writeXmlMatrix(xmlNodePtr parent,p3d_matrix4 m)
{
  char str[1000];
  
  for(int i = 0; i<4; i++)
  {
    for(int j = 0; j<4; j++)
    {
      sprintf(str, "%f.6", m[i][j]);
      if (j < 3) {
        sprintf(str, " ");
      }
    }
    if (i<3) {
      sprintf(str, "\n");
    }
  }
  
  xmlNewChild(parent, NULL, xmlCharStrdup("matrix"), xmlCharStrdup(str));
}

//! Write an IK sol to a file
//! 
void writeXmlIkSol(xmlNodePtr parent,p3d_rob* rob, int* iksol){
  xmlNodePtr iksolNode = NULL;
  char str[80];
  
  if(iksol){
    iksolNode = xmlNewChild(parent, NULL, xmlCharStrdup("iksol"), NULL);
    sprintf(str, "%d", rob->cntrt_manager->ncntrts);
    xmlNewProp (iksolNode, xmlCharStrdup("num"), xmlCharStrdup(str));
    for(int i = 0; i < rob->cntrt_manager->ncntrts; i++){
      sprintf(str, "%d", iksol[i]);
      xmlNewChild(iksolNode, NULL, xmlCharStrdup("cntrtSol"), xmlCharStrdup(str));
    }
  }
}

//! Write a config to a file
//! 
void writeXmlConfig(xmlNodePtr parent, p3d_rob* rob, configPt q ){
  xmlNodePtr config = NULL;
  char str[80];
  configPt q_deg;
  
  config = xmlNewChild(parent, NULL, xmlCharStrdup("config"), NULL);
  q_deg = p3d_copy_config_rad_to_deg(rob,q);
  sprintf(str, "%d", rob->nb_dof);
  xmlNewProp (config, xmlCharStrdup("num"), xmlCharStrdup(str));
  for(int i=0; i < rob->nb_dof; i++){
    sprintf(str, "%f", q_deg[i]);
    xmlNewChild(config, NULL, xmlCharStrdup("dofVal"), xmlCharStrdup(str));
  }
  p3d_destroy_config(rob, q_deg);
}

//! Write a constraint to a file
//! 
void writeXmlCntrts(p3d_rob* rob, p3d_cntrt* ct, xmlNodePtr parent)
{
  xmlNodePtr xmlCntrts = NULL, tmp = NULL , tmp2 = NULL;
  char str[80];
  
  xmlCntrts = xmlNewChild(parent, NULL, xmlCharStrdup("Constraint"),NULL);
  
  xmlNewProp(xmlCntrts, xmlCharStrdup("Name"), xmlCharStrdup(ct->namecntrt));
  
  // Passive Joints
  tmp = xmlNewChild(xmlCntrts, NULL, xmlCharStrdup("PasJnts"),NULL);
  sprintf(str, "%d", ct->npasjnts);
  xmlNewProp(xmlCntrts, xmlCharStrdup("Number"), xmlCharStrdup(str));
  for(int i = 0; i<ct->npasjnts; i++)
  {
    sprintf(str, " %d", ct->pasjnts[i]->num);
    xmlNewChild(tmp, NULL, xmlCharStrdup("Joint"),xmlCharStrdup(str));
  }
  
  // Active Joints
  tmp = xmlNewChild(xmlCntrts, NULL, xmlCharStrdup("ActJnts"),NULL);
  sprintf(str, "%d", ct->nactjnts);
  xmlNewProp(xmlCntrts, xmlCharStrdup("Number"), xmlCharStrdup(str));
  for(int i = 0; i<ct->nactjnts; i++)
  {
    sprintf(str, " %d", ct->actjnts[i]->num);
    xmlNewChild(tmp, NULL, xmlCharStrdup("Joint"),xmlCharStrdup(str));
  }
  
  // Double Argument
  tmp = xmlNewChild(xmlCntrts, NULL, xmlCharStrdup("DVal"),NULL);
  sprintf(str, "%d", ct->ndval);
  xmlNewProp(xmlCntrts, xmlCharStrdup("Number"), xmlCharStrdup(str));
  for(int i = 0; i<ct->ndval; i++)
  {
    sprintf(str, " %f", ct->argu_d[i]);
    xmlNewChild(tmp, NULL, xmlCharStrdup("Value"),xmlCharStrdup(str));
  }
  
  // Integer Argument
  tmp = xmlNewChild(xmlCntrts, NULL, xmlCharStrdup("ActJnts"),NULL);
  sprintf(str, "%d", ct->nival);
  xmlNewProp(xmlCntrts, xmlCharStrdup("Number"), xmlCharStrdup(str));
  for(int i = 0; i<ct->nival; i++)
  {
    sprintf(str, " %d", ct->argu_i[i]);
    xmlNewChild(tmp, NULL, xmlCharStrdup("Value"),xmlCharStrdup(str));
  }
  
  // TAtt Matrix
  tmp = xmlNewChild(xmlCntrts, NULL, xmlCharStrdup("Tatt"),NULL);
  writeXmlMatrix(tmp,ct->Tatt);
  
  // TAtt2 Matrix
  tmp = xmlNewChild(xmlCntrts, NULL, xmlCharStrdup("Tatt"),NULL);
  writeXmlMatrix(tmp,ct->Tatt2);
  
}

//! Write a localpath to a file
//! 
xmlNodePtr writeXmlLocalpath(xmlNodePtr parent, p3d_rob* rob, p3d_localpath * localpath ) 
{
	xmlNodePtr xmlLocalpath = NULL, tmp = NULL;
	char str[80];
  
	xmlLocalpath = xmlNewChild(parent, NULL, xmlCharStrdup("localpath"),NULL);
	xmlNewProp(xmlLocalpath, xmlCharStrdup("type"), xmlCharStrdup(p3d_local_getname_planner(localpath->type_lp)));
  sprintf(str, "%f", localpath->range_param);
	xmlNewProp(xmlLocalpath, xmlCharStrdup("size"), xmlCharStrdup(str));
  
#ifdef MULTILOCALPATH
	if (localpath->type_lp == MULTI_LOCALPATH) {
		sprintf(str, "%d", rob->mlp->nblpGp);
		xmlNewProp(xmlLocalpath, xmlCharStrdup("nbGroup"), xmlCharStrdup(str));
		for(int i=0; i < rob->mlp->nblpGp; i++){
			tmp = xmlNewChild(xmlLocalpath, NULL, xmlCharStrdup("sub_localpath"),NULL);
			sprintf(str, "%d", p3d_multiLocalPath_get_value_groupToPlan(rob, i));
			xmlNewProp(tmp, xmlCharStrdup("groupActivated"), xmlCharStrdup(str));
			xmlNewProp(tmp, xmlCharStrdup("type"), xmlCharStrdup(p3d_local_getname_planner(rob->mlp->mlpJoints[i]->lplType)));
			if ((rob->mlp->mlpJoints[i]->lplType == REEDS_SHEPP) || (rob->mlp->mlpJoints[i]->lplType == DUBINS)) {
				plm_reeds_shepp_str rs_paramPt = lm_get_reeds_shepp_lm_param(rob);
				double radius = 0;
				if (rs_paramPt == NULL) {
					radius = -1;
				} else {
					radius = rs_paramPt->radius;
				}
				sprintf(str, "%f", radius);
				xmlNewProp(tmp, xmlCharStrdup("radius"), xmlCharStrdup(str));
			}
		}
	} 
#endif
  
  if ((localpath->type_lp == REEDS_SHEPP) || (localpath->type_lp == DUBINS)) {
    plm_reeds_shepp_str rs_paramPt = lm_get_reeds_shepp_lm_param(rob);
    double radius = 0;
    if (rs_paramPt == NULL) {
      radius = -1;
    } else {
      radius = rs_paramPt->radius;
    }
    sprintf(str, "%f", radius);
    xmlNewProp(tmp, xmlCharStrdup("radius"), xmlCharStrdup(str));
  }
  
  tmp = xmlNewChild(xmlLocalpath, NULL, xmlCharStrdup("Object"),NULL);
  
  sprintf(str, "%d", localpath->isCarryingObject);
  xmlNewProp(tmp, xmlCharStrdup("isCarryingObject"), xmlCharStrdup(str));

  for (int i=0; i<localpath->isCarryingObject; i++) 
  {
    xmlNewProp(tmp, xmlCharStrdup("ObjectName"), xmlCharStrdup(localpath->carriedObject[i]->name));
  }
  

  tmp = xmlNewChild(xmlLocalpath, NULL, xmlCharStrdup("Constraints"), NULL);
  
  sprintf(str, "%d",localpath->nbActiveCntrts );
  xmlNewProp(tmp, xmlCharStrdup("nbActive"), xmlCharStrdup(str));
  
  for (int i=0; i<localpath->nbActiveCntrts; i++)
  {
    writeXmlCntrts(rob,rob->cntrt_manager->cntrts[localpath->activeCntrts[i]],tmp);
  }
  
  return xmlLocalpath;
}

