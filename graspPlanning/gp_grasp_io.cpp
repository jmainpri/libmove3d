/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */

#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Graphic-pkg.h"
#include "GraspPlanning-pkg.h"
#include <math.h>
#include <string>
#include <sstream>
#include <fstream>

#include <libxml2/libxml/xmlreader.h>

std::string getNodeString(xmlDocPtr doc, xmlNodePtr node);
void warningMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message);
void formatErrorMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message);
void elementMissingMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message);
bool gpParseElement(xmlDocPtr doc, xmlNodePtr entry_node, std::string element, gpElementParserData &data);
bool gpParseContact(xmlDocPtr doc, xmlNodePtr entry_node, gpGraspParserData &data);
bool gpParseGrasp(xmlDocPtr doc, xmlNodePtr entry_node, gpGraspParserData &data);

//! @ingroup graspIO 
//! Saves a grasp list as a file in XML format.
//! \param graspList the grasp list to save
//! \param filename the file where to save the list in (if it has no .xml extension, the extension will be added)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSave_grasp_list(std::list<gpGrasp> &graspList, std::string filename)
{
  unsigned int i;
  size_t pos;
  gpHand_type hand_type;
  std::string object_name;
  FILE *file= NULL;
  std::list<gpGrasp>::iterator grasp;
  time_t rawtime;
  struct tm * timeinfo;

  if(graspList.empty())
  {
    printf("%s: %d: gpSave_grasp_list(): the grasp list is empty.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  object_name= graspList.front().object_name;
  hand_type= graspList.front().hand_type;

  pos= filename.find(".xml"); 
  if(pos!=filename.length()-4)
  {
    filename+= ".xml";
  }    

  file= fopen(filename.c_str(),"w");

  if(file==NULL)
  {
    printf("%s: %d: gpSave_grasp_list(): could not open file \"%s\".\n",__FILE__,__LINE__,filename.c_str());
    return GP_ERROR;
  }

  time(&rawtime);
  timeinfo= localtime(&rawtime);

  fprintf(file, "<!-- grasp list for object \"%s\" with \"%s\" hand \n", object_name.c_str(), (gpHand_type_to_string(hand_type)).c_str());
  fprintf(file, " creation date: %s -->\n", asctime(timeinfo));
  fprintf(file, "<grasp_list nb_elements=\"%d\"> \n", (int)graspList.size());
  fprintf(file, "  <version> %s </version>\n", GP_VERSION);
  fprintf(file, "  <object_name> %s </object_name> \n", object_name.c_str());
  fprintf(file, "  <hand_type> %s </hand_type> \n", (gpHand_type_to_string(hand_type)).c_str());
//   fprintf(file, "  <nb_vertices> %s </nb_vertices> \n", (gpHand_type_to_string(hand_type)).c_str());

  for(grasp=graspList.begin(); grasp!=graspList.end(); grasp++)
  {
    fprintf(file, "  <grasp ID=\"%d\" nb_contacts=\"%d\" nb_dofs=\"%d\"> \n", grasp->ID, (int)grasp->contacts.size(), (int)grasp->config.size());
    if(grasp->autoGen==true)
    {
      fprintf(file, "    <autoGen> true </autoGen>\n");
    }
    else
    {
      fprintf(file, "    <autoGen> false </autoGen>\n");
    }
    fprintf(file, "    <ID> %d </ID>\n", grasp->ID);
    fprintf(file, "    <object_name> %s </object_name> \n", grasp->object_name.c_str());
    fprintf(file, "    <hand_type> %s </hand_type> \n", gpHand_type_to_string(grasp->hand_type).c_str());
    fprintf(file, "    <handID> %d </handID>\n", grasp->handID);
    fprintf(file, "    <stability> %f </stability> \n", grasp->stability);
    fprintf(file, "    <quality> %f </quality> \n", grasp->quality);
    fprintf(file, "    <frame>   %f %f %f %f \n", grasp->frame[0][0], grasp->frame[0][1], grasp->frame[0][2], grasp->frame[0][3]);
    fprintf(file, "              %f %f %f %f \n", grasp->frame[1][0], grasp->frame[1][1],grasp-> frame[1][2], grasp->frame[1][3]);
    fprintf(file, "              %f %f %f %f \n", grasp->frame[2][0], grasp->frame[2][1], grasp->frame[2][2], grasp->frame[2][3]);
    fprintf(file, "              %f %f %f %f \n", grasp->frame[3][0], grasp->frame[3][1], grasp->frame[3][2], grasp->frame[3][3]);
    fprintf(file, "    </frame> \n");

    for(i=0; i<grasp->contacts.size(); i++)
    {
      fprintf(file, "    <contact> \n");
      fprintf(file, "      <position> %f %f %f </position> \n", grasp->contacts[i].position[0], grasp->contacts[i].position[1], grasp->contacts[i].position[2]);
      fprintf(file, "      <normal> %f %f %f </normal> \n", grasp->contacts[i].normal[0], grasp->contacts[i].normal[1], grasp->contacts[i].normal[2]);
      fprintf(file, "      <force_direction> %f %f %f </force_direction> \n", grasp->contacts[i].forceDirection[0], grasp->contacts[i].forceDirection[1], grasp->contacts[i].forceDirection[2]);
      fprintf(file, "      <barycentric_cordinates> %f %f %f </barycentric_cordinates> \n", grasp->contacts[i].baryCoords[0], grasp->contacts[i].baryCoords[1], grasp->contacts[i].baryCoords[2]);
      fprintf(file, "      <fingerID> %d </fingerID> \n", grasp->contacts[i].fingerID);
      if(grasp->contacts[i].surface!=NULL)
      {  fprintf(file, "      <surface> %s </surface> \n", grasp->contacts[i].surface->name);  }
      fprintf(file, "      <face> %d </face> \n", grasp->contacts[i].face);
      fprintf(file, "      <friction_coefficient> %f </friction_coefficient> \n", grasp->contacts[i].mu);
      fprintf(file, "      <curvature> %f </curvature> \n", grasp->contacts[i].curvature);
      fprintf(file, "    </contact> \n");
    }

    fprintf(file, "    <configuration> \n");
    for(i=0; i<grasp->config.size(); i++)
    {
      fprintf(file, "  %f\n", grasp->config[i]);
    }
    fprintf(file, "    </configuration> \n");

    fprintf(file, "    <open_configuration> \n");
    for(i=0; i<grasp->openConfig.size(); i++)
    {
      fprintf(file, "  %f\n", grasp->openConfig[i]);
    }
    fprintf(file, "    </open_configuration> \n");


    fprintf(file, "  </grasp> \n");
  }
  fprintf(file, "</grasp_list> \n");

  fclose(file);

  return GP_OK;
}

//! @ingroup graspIO 
//! Converts the char string of the given node to a std::string after removing all newline characters
//! as well as space characters at the end of the string.
std::string getNodeString(xmlDocPtr doc, xmlNodePtr node)
{
  size_t pos;
  xmlChar *key;
  std::string  message, word;
  key= xmlNodeListGetString(doc, node->xmlChildrenNode, 1);
  std::string text((char *) key);
  xmlFree(key);

  //Remove all the newline characters for UNIX file (\n) at the end of the line:
  pos= text.find("\n");
  while(pos!=std::string::npos)
  {
    text.replace(pos, 1, " ");
    pos= text.find("\n");
  }

  //Remove all the newlines character for WINDOWS/DOS file (\r\n) at the end of the line:
  pos= text.find("\r");
  while(pos!=std::string::npos)
  {
    text.replace(pos, 1, " ");
    pos= text.find("\r");
  }

  //Remove all the spaces at the end of the line:
  pos= text.find_last_not_of(' ');
  text= text.substr(0, pos+1);

  return text;
}


//! @ingroup graspIO 
void warningMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message)
{
   printf("At line %d in %s: warning: in an element of <%s>: \n", line_number, URL, element_name);
   printf(" %s\n\n", message.c_str());
   return;
}

//! @ingroup graspIO 
void formatErrorMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message)
{
   printf("At line %d in %s: format error in the data of an element <%s>.\n", line_number, URL, element_name);
   printf(" %s\n\n", message.c_str());

   return;
}

//! @ingroup graspIO 
void elementMissingMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message)
{
   printf("At line %d in %s: A sub-element of element <%s> is missing.\n", line_number, URL, element_name);
   printf(" %s\n\n", message.c_str());

   return;
}



//! @ingroup graspIO 
//! Reads the content of the given node and fills the appropriate field in the gpElementParserData structure.
//! This function is used for simple element (with no chidren nodes).
//! \param doc pointer to the structure used by libxml2 to parse the XML document
//! \param entry_node the node whose children will be browsed
//! \param element the name of the element that will be looked for among the entry node children
//! \param data the data structure that will be used to store the read information
//! \return true if the searched element was found, false otherwise
bool gpParseElement(xmlDocPtr doc, xmlNodePtr entry_node, std::string element, gpElementParserData &data)
{
  bool result;
  double x;
  std::string text;
  std::string message, word;
  std::istringstream iss;
  xmlNodePtr cur= entry_node->xmlChildrenNode;
	
  iss.clear();

  for(cur= entry_node->xmlChildrenNode; cur!=NULL; cur= cur->next)
  {
    if( !xmlStrcmp(cur->name, (xmlChar*) element.c_str() ) )
    {
       iss.str(getNodeString(doc, cur));

       if(element=="version")
       { 
         result= (iss >> data.version);
         if( !result || !iss.eof() )
         {
           message= "Usage: <version> version of the grasp planner </version>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="autoGen")
       { 
         result= (iss >> data.autoGen);
         if( !result || !iss.eof() )
         {
           message= "Usage: <autoGen> wether or not the grasp was generated automatically (true or false) </autoGen>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="object_name")
       { 
         result= (iss >> data.object_name);
         if( !result || !iss.eof() )
         {
           message= "Usage: <object_name> object name </object_name>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="frame")
       { 
         result= (iss >> data.frame[0][0] >> data.frame[0][1] >> data.frame[0][2] >> data.frame[0][3] >> data.frame[1][0] >> data.frame[1][1] >> data.frame[1][2] >> data.frame[1][3] >>
         data.frame[2][0] >> data.frame[2][1] >> data.frame[2][2] >> data.frame[2][3] >> 
         data.frame[3][0] >> data.frame[3][1] >> data.frame[3][2] >> data.frame[3][3] );

         if( !result || !iss.eof() )
         {
           message= "Usage: <frame> R11 R12 R13 tx R21 R22 R23 ty R31 R32 R33 tz </frame>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         data.frame[3][0]= data.frame[3][1]= data.frame[3][2]= 0.0;
         data.frame[3][3]= 1.0;
         return true;
       }

       if(element=="hand_type")
       {
         result= (iss >> data.hand_type );
         if( !result || !iss.eof() )
         {
           message= "Usage: <hand_type> hand or gripper type </hand_type>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="position")
       {
         result= (iss >> data.position[0] >> data.position[1] >> data.position[2] );
         if( !result || !iss.eof() )
         {
           message= "Usage: <position> x y z </position>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="normal")
       {
         result= (iss >> data.normal[0] >> data.normal[1] >> data.normal[2] );
         if( !result || !iss.eof() )
         {
           message= "Usage: <normal> x y z </normal>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="force_direction")
       {
         result= (iss >> data.force_direction[0] >> data.force_direction[1] >> data.force_direction[2] );
         if( !result || !iss.eof() )
         {
           message= "Usage: <force_direction> x y z </force_direction>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="barycentric_cordinates")
       {
         result= (iss >> data.baryCoords[0] >> data.baryCoords[1] >> data.baryCoords[2] );
         if( !result || !iss.eof() )
         {
           message= "Usage: <barycentric_cordinates> alpha beta gamma </barycentric_cordinates>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="stability")
       {
         result= (iss >> data.stability );
         if( !result || !iss.eof() )
         {
           message= "Usage: <stability> grasp stability score </stability>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="quality")
       {
         result= (iss >> data.quality );
         if( !result || !iss.eof() )
         {
           message= "Usage: <quality> grasp quality score </quality>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="friction_coefficient")
       {
         result= (iss >> data.friction_coefficient );
         if( !result || !iss.eof() )
         {
           message= "Usage: <friction_coefficient> friction coefficient of the contact </friction_coefficient>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="curvature")
       {
         result= (iss >> data.curvature );
         if( !result || !iss.eof() )
         {
           message= "Usage: <curvature> surface curvature at the contact </curvature>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }


       if(element=="ID")
       {
         result= (iss >> data.ID );
         if( !result || !iss.eof() )
         {
           message= "Usage: <ID> grasp ID </ID>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="handID")
       {
         result= (iss >> data.handID );
         if( !result || !iss.eof() )
         {
           message= "Usage: <handID> hand ID </handID>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="body_index")
       {
         result= (iss >> data.body_index );
         if( !result || !iss.eof() )
         {
           message= "Usage: <body_index> index of the grasped body (in the array of p3d_obj* of the object (a freeflyer robot)) </body_index>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="fingerID")
       {
         result= (iss >> data.fingerID );
         if( !result || !iss.eof() )
         {
           message= "Usage: <fingerID> finger ID </fingerID>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="face")
       {
         result= (iss >> data.face );
         if( !result || !iss.eof() )
         {
           message= "Usage: <face> face index (starting from 0) </face>.";
           formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
           return false;
         } 
         return true;
       }

       if(element=="configuration")
       {
          data.configuration.clear();
          while(!iss.eof())
          {
            result= (iss >> x );
            if(!result)
            {
              message= "Usage: <configuration> hand's joint parameters (angles are given in radians, lengths in meters) </configuration>.";
              formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
              return false;
            } 
            else
            {
               data.configuration.push_back(x);  
            }
          }
          return true;
       }

       if(element=="open_configuration")
       {
          data.open_configuration.clear();
          while(!iss.eof())
          {
            result= (iss >> x );
            if(!result)
            {
              message= "Usage: <open_configuration> hand's joint parameters (angles are given in radians, lengths in meters) </open_configuration>.";
              formatErrorMessage((int) xmlGetLineNo(cur), doc->URL, cur->name, message);
              return false;
            } 
            else
            {
               data.open_configuration.push_back(x);  
            }
          }
          return true;
       }
    }   
  }

//   printf("%s: %d: There is no element called %s\n",__FILE__,__LINE__,element.c_str());

  return false;
}


//! @ingroup graspIO 
//! Reads the content of a <contact> node and fills the gpContactParserData structure.
//! \param doc pointer to the structure used by libxml2 to parse the XML document
//! \param entry_node the node whose children will be browsed
//! \param data the data structure that will be used to store all the read information about the body
//! \return true if all the mandatory elements were found, false otherwise
bool gpParseContact(xmlDocPtr doc, xmlNodePtr entry_node, gpContactParserData &data)
{
  xmlNodePtr cur= entry_node;
  gpElementParserData elementData;
  std::string message;

  if(!gpParseElement(doc, entry_node, "position", elementData)) 
  {  
    message= "Element <contact> should have an element <position>.";
    elementMissingMessage((int) xmlGetLineNo(cur), doc->URL, entry_node->name, message);
    return false;
  }  
  else
  {
    p3d_vectCopy(elementData.position, data.position);
  }

  if(!gpParseElement(doc, entry_node, "normal", elementData)) 
  {  
    message= "Element <contact> should have an element <normal>.";
    elementMissingMessage((int) xmlGetLineNo(cur), doc->URL, entry_node->name, message);
    return false;
  }  
  else
  {
    p3d_vectCopy(elementData.normal, data.normal);
  }

  if(!gpParseElement(doc, entry_node, "force_direction", elementData)) 
  {  
    message= "Element <force_direction> should have an element <force_direction>.";
    elementMissingMessage((int) xmlGetLineNo(cur), doc->URL, entry_node->name, message);
    return false;
  }  
  else
  {
    p3d_vectCopy(elementData.force_direction, data.force_direction);
  }

  if(!gpParseElement(doc, entry_node, "barycentric_cordinates", elementData)) 
  {  
    message= "Element <contact> should have an element <barycentric_cordinates>.";
    elementMissingMessage((int) xmlGetLineNo(cur), doc->URL, entry_node->name, message);
    return false;
  }  
  else
  {
    p3d_vectCopy(elementData.baryCoords, data.baryCoords);
  }

  if(!gpParseElement(doc, entry_node, "fingerID", elementData)) 
  {  
    message= "Element <contact> should have an element <fingerID>.";
    elementMissingMessage((int) xmlGetLineNo(cur), doc->URL, entry_node->name, message);
    return false;
  }  
  else
  {
    data.fingerID= elementData.fingerID;
  }

  if(!gpParseElement(doc, entry_node, "face", elementData)) 
  {  
    message= "Element <contact> should have an element <face>.";
    elementMissingMessage((int) xmlGetLineNo(cur), doc->URL, entry_node->name, message);
    return false;
  }  
  else
  {
    data.face= elementData.face;
  }

  if(!gpParseElement(doc, entry_node, "friction_coefficient", elementData)) 
  {  
    message= "Element <contact> should have an element <friction_coefficient>.";
    elementMissingMessage((int) xmlGetLineNo(cur), doc->URL, entry_node->name, message);
    return false;
  }  
  else
  {
    data.friction_coefficient= elementData.friction_coefficient;
  }

  if(!gpParseElement(doc, entry_node, "curvature", elementData)) 
  {  
    message= "Element <curvature> should have an element <curvature>.";
    elementMissingMessage((int) xmlGetLineNo(cur), doc->URL, entry_node->name, message);
    return false;
  }  
  else
  {
    data.curvature= elementData.curvature;
  }


  return true;
}

//! @ingroup graspIO 
//! Reads the content of a <grasp> node and fills the gpGraspParserData structure.
//! \param doc pointer to the structure used by libxml2 to parse the XML document
//! \param entry_node the node whose children will be browsed
//! \param data the data structure that will be used to store all the read information about the body
//! \return true if all the mandatory elements were found, false otherwise
bool gpParseGrasp(xmlDocPtr doc, xmlNodePtr entry_node, gpGraspParserData &data)
{
  xmlNodePtr cur= entry_node;
  gpElementParserData elementData;
  gpContactParserData contactData;
  std::string message;
  gpContact contact;

  if(!gpParseElement(doc, entry_node, "object_name", elementData)) 
  {  
    message= "Element <grasp> should have an element <object_name>.";
    elementMissingMessage((int) xmlGetLineNo(cur), doc->URL, entry_node->name, message);
    return false;
  }  
  else
  {
    data.object_name= elementData.object_name;
  }

  if(!gpParseElement(doc, entry_node, "hand_type", elementData)) 
  {  
    message= "Element <grasp> should have an element <hand_type>.";
    elementMissingMessage((int) xmlGetLineNo(cur), doc->URL, entry_node->name, message);
    return false;
  }  
  else
  {
    if(elementData.hand_type=="GP_GRIPPER")
    {  data.hand_type= GP_GRIPPER;  }
    else if(elementData.hand_type=="GP_PR2_GRIPPER")
    {  data.hand_type= GP_PR2_GRIPPER;  }
    else if(elementData.hand_type=="GP_SAHAND_RIGHT")
    {  data.hand_type= GP_SAHAND_RIGHT;  }
    else if(elementData.hand_type=="GP_SAHAND_LEFT")
    {  data.hand_type= GP_SAHAND_LEFT;    }
    else
    {
      message= "Usage: <hand_type> type of the robot gripper or hand (GP_GRIPPER, GP_PR2_GRIPPER, GP_SAHAND_RIGHT or GP_SAHAND_LEFT) </hand_type>.";
      formatErrorMessage((int) xmlGetLineNo(entry_node), doc->URL, entry_node->name, message);
      return false;
    }
  }

  if(gpParseElement(doc, entry_node, "ID", elementData)) 
  {
    data.ID= elementData.ID;
  }
  else
  {
    data.ID= 0;
  }

  if(gpParseElement(doc, entry_node, "autoGen", elementData)) 
  {
    if(elementData.autoGen=="true")
    { data.autoGen= true; }
    else if(elementData.autoGen=="false")
    {  data.autoGen= false;  }
    else
    {
      message= "Usage: <autoGen> wether or not the grasp was generated automatically (true or false) </autoGen>.";
      formatErrorMessage((int) xmlGetLineNo(entry_node), doc->URL, entry_node->name, message);
      return false;
    }
  }
  else
  {
    data.autoGen= true;
  }

  if(gpParseElement(doc, entry_node, "handID", elementData)) 
  {
    data.handID= elementData.handID;
  }
  else
  {
    data.handID= 0;
  }


  if(gpParseElement(doc, entry_node, "body_index", elementData)) 
  {
    data.body_index= elementData.body_index;
  }
  else
  {
    data.body_index= 0;
  }

  if(!gpParseElement(doc, entry_node, "stability", elementData)) 
  {
    message= "Usage: <grasp> should have a </stability> element.";
    warningMessage((int) xmlGetLineNo(entry_node), doc->URL, entry_node->name, message);
  }
  else
  {
    data.stability= elementData.stability;
  }

  if(!gpParseElement(doc, entry_node, "quality", elementData)) 
  {
    message= "Usage: <grasp> should have a </quality> element.";
    warningMessage((int) xmlGetLineNo(entry_node), doc->URL, entry_node->name, message);
  }
  else
  {
    data.quality= elementData.quality;
  }


  if(!gpParseElement(doc, entry_node, "frame", elementData)) 
  {
    message= "Usage: <grasp> should have a </frame> element.";
    elementMissingMessage((int) xmlGetLineNo(entry_node), doc->URL, entry_node->name, message);
    return false;
  }
  else
  {
    p3d_mat4Copy(elementData.frame, data.frame);
  }

  data.configuration.clear();
  if(!gpParseElement(doc, entry_node, "configuration", elementData)) 
  {
    message= "Usage: <grasp> should have a </configuration> element.";
    elementMissingMessage((int) xmlGetLineNo(entry_node), doc->URL, entry_node->name, message);
    return false;
  }
  else
  {
    data.configuration= elementData.configuration;
  }

  data.open_configuration.clear();
  if(!gpParseElement(doc, entry_node, "open_configuration", elementData)) 
  {
    data.open_configuration= data.configuration;
  }
  else
  {
    data.open_configuration= elementData.open_configuration;
  }

  // Read all the contact elements:
  data.contacts.clear();
  for(cur= entry_node->xmlChildrenNode; cur!=NULL; cur= cur->next)
  {
    if((!xmlStrcmp(cur->name, (const xmlChar *)"contact")))
    { 
      if(gpParseContact(doc, cur, contactData))
      { 
         contact.fingerID= contactData.fingerID;
         contact.face= contactData.face;
         p3d_vectCopy(contactData.position, contact.position);
         p3d_vectCopy(contactData.normal, contact.normal);
         p3d_vectCopy(contactData.force_direction, contact.forceDirection);
         p3d_vectCopy(contactData.baryCoords, contact.baryCoords);
         contact.mu= contactData.friction_coefficient;
         contact.curvature= contactData.curvature;
         data.contacts.push_back(contact);
      }
    }
  }
  

  return true;
}

//! @ingroup graspIO 
//! Parses an XML file describing a grasp list.
//! \param filename name of the XML file
//! \param graspList the grasp list that will be filled with the content of the file
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpLoad_grasp_list(std::string filename, std::list<gpGrasp> &graspList)
{
  xmlDocPtr doc;
  xmlNodePtr root, cur;
  std::string text, message, version= "undefined";
  gpElementParserData elementData;
  gpGraspParserData graspData;
  gpGrasp grasp;
  std::list<gpContact>::iterator contact_iter;
  std::list<double>::iterator q_iter;
  p3d_rob *object= NULL;

  xmlLineNumbersDefault(1);
  doc= xmlParseFile(filename.c_str());

  if(doc==NULL)
  {
    printf("%s: %d: gpLoad_grasp_list(): document \"%s\" does not exist or was not parsed successfully by libxml2.\n", __FILE__, __LINE__, filename.c_str());
    return GP_ERROR;
  }
	
  root = xmlDocGetRootElement(doc);
	
  if(root==NULL)
  {
    printf("%s: %d: gpLoad_grasp_list(): document \"%s\" is empty.\n", __FILE__, __LINE__, filename.c_str());
    xmlFreeDoc(doc);
    return GP_ERROR;
  }
	

  if(xmlStrcmp(root->name, (const xmlChar *) "grasp_list"))
  {
    printf("%s: %d: gpLoad_grasp_list(): document \"%s\" is of the wrong type, root node != <grasp_list>.\n", __FILE__, __LINE__, filename.c_str());
    xmlFreeDoc(doc);
    return GP_ERROR;
  }

  //first check the version:
  if(gpParseElement(doc, root, "version", elementData))
  {   
     version= elementData.version;
  }

  if(version!=std::string(GP_VERSION))
  {
    printf("gpLoad_grasp_list(): the document version of \"%s\" (%s) is  obsolete (it should be \"%s\"). The file will not be read further. Delete the file and recompute a grasp list.\n", filename.c_str(), version.c_str(), (char *)(GP_VERSION));
    xmlFreeDoc(doc);
    return GP_ERROR;
  }


  //Read all the grasp elements:
  for(cur= root->xmlChildrenNode; cur!=NULL; cur= cur->next)
  {
    if(!xmlStrcmp(cur->name, (const xmlChar *)"grasp"))
    { 
      if(gpParseGrasp(doc, cur, graspData))
      { 
         grasp.object_name= graspData.object_name;
         grasp.hand_type= graspData.hand_type;
         grasp.ID= graspData.ID;
         grasp.handID= graspData.handID;
         grasp.stability= graspData.stability;
         grasp.quality= graspData.quality;
         p3d_mat4Copy(graspData.frame, grasp.frame);
         object= p3d_get_robot_by_name((char *)(grasp.object_name.c_str()));
         if(object==NULL)
         {
           printf("A robot named \"%s\" is required by a grasp.\n",grasp.object_name.c_str());
         }
         grasp.object= object;
         grasp.contacts.clear();
         grasp.contacts.reserve(graspData.contacts.size());
         for(contact_iter=graspData.contacts.begin(); contact_iter!=graspData.contacts.end(); contact_iter++)
         {
           if(object!=NULL) 
           { contact_iter->surface= object->o[0]->pol[0]->poly; }
           grasp.contacts.push_back(*contact_iter);
         }
         grasp.config.clear();
         grasp.config.reserve(graspData.configuration.size());
         for(q_iter=graspData.configuration.begin(); q_iter!=graspData.configuration.end(); q_iter++)
         {
           grasp.config.push_back(*q_iter);
         }
         grasp.openConfig.clear();
         grasp.openConfig.reserve(graspData.open_configuration.size());
         for(q_iter=graspData.open_configuration.begin(); q_iter!=graspData.open_configuration.end(); q_iter++)
         {
           grasp.openConfig.push_back(*q_iter);
         }

         graspList.push_back(grasp);
      }
    }
  }

  xmlFreeDoc(doc);

  return GP_OK;
}

//! @ingroup graspIO 
//! Checks the validity of a grasp list.
//! \param graspList the grasp list to check
//! \param objectName name of the object (a freeflying robot) we  load the list for
//! \return GP_OK in case of success (valid list), GP_ERROR otherwise
int gpCheck_grasp_list_validity(std::list<gpGrasp> &graspList,  std::string objectName)
{
  unsigned int i;
  p3d_rob *object= NULL;
  p3d_rob *list_object= NULL;
  std::string object_name;
  std::list<gpGrasp>::iterator igrasp;

  if(graspList.empty())
  {
    return GP_ERROR;
  }

  object= p3d_get_robot_by_name(objectName.c_str());

  if(object==NULL)
  {
    printf("%s: %d: gpCheck_grasp_list_validity(): there is no robot named \"%s\".\n",__FILE__,__LINE__,objectName.c_str());
    return GP_ERROR;
  }

  list_object= graspList.front().object;

  if(list_object==NULL)
  { 
    return GP_ERROR;
  }

  object_name= graspList.front().object_name;
  for(igrasp=graspList.begin(); igrasp!=graspList.end(); igrasp++)
  {
    if(igrasp->object_name!=object_name)
    {
       printf("%s: %d: gpCheck_grasp_list_validity(): the grasps of the list were not all computed for the same object.\n",__FILE__,__LINE__);
       return GP_ERROR;
    }
  } 

  for(igrasp=graspList.begin(); igrasp!=graspList.end(); igrasp++)
  {
    if(igrasp->object_name!=objectName)
    {
       printf("%s: %d: gpCheck_grasp_list_validity(): some grasps of the list have not been computed for the input object.\n",__FILE__,__LINE__);
       return GP_ERROR;
    }

    for(i=0; i<igrasp->contacts.size(); ++i)
    {
      if(igrasp->contacts[i].face > object->o[0]->pol[0]->poly->nb_faces)
      {
        printf("face %d\n",igrasp->contacts[i].face);
        printf("%s: %d: gpCheck_grasp_list_validity(): a contact of an element of the grasp list has an inconsistent face index.\n",__FILE__,__LINE__);
       return GP_ERROR;
      }
    }
  } 


  return GP_OK;
}

//! WIP
int gpInvert_axis(std::string inputFile, std::string outputFile, p3d_matrix4 T)
{
  p3d_vector3 p1, p2, r1, r2;
  p3d_matrix4 T1, T2;
  std::ifstream in;
  std::ofstream out;
  char line[200], str[100];
//  float x, y, z, r11, r12, r13, r21, r22, r23, r31, r32, r33;
  bool relative= false;

  in.open(inputFile.c_str());
  if(in==NULL)
  {
    printf("fichier %s introuvable\n", inputFile.c_str());
    return GP_ERROR;
  }

  out.open(outputFile.c_str(), std::ios::trunc);


  while(in.eof()==false)
  {
     in.getline(line, 200);
     sscanf(line, "%s ", str);

     if(strcmp(str, "p3d_add_desc_vert")==0)
     {
//         sscanf(line, "%s %f %f %f", str, &x, &y, &z);
        sscanf(line, "%s %lf %lf %lf", str, &p1[0], &p1[1], &p1[2]);
        p3d_xformVect(T, p1, p2);
// printf("%f %f %f -> %f %f %f \n",p1[0], p1[1], p1[2],p2[0], p2[1], p2[2]);
//         switch(axis)  
//         {
//           case 1:
//              x= -x;
//           break;
//           case 2:
//              y= -y;
//           break;
//           case 3:
//              z= -z;
//           break;
//         } 
//         out << "p3d_add_desc_vert " << x << " " << y << " " << z << std::endl;
        out << "p3d_add_desc_vert " << p2[0] << " " << p2[1] << " " << p2[2] << std::endl;
        continue;
     }

     if(strcmp(str, "p3d_set_prim_pos_by_mat")==0)
     {
//         sscanf(line, "%s %f %f %f %f %f %f %f %f %f %f %f %f", str, &r11, &r12, &r13, &x, &r21, &r22, &r23, &y, &r31, &r32, &r33, &z);
        sscanf(line, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", str, &T1[0][0], &T1[0][1], &T1[0][2], &T1[0][3], &T1[1][0], &T1[1][1], &T1[1][2], &T1[2][3], &T1[2][0], &T1[2][1], &T1[2][2], &T1[2][3]);

//         switch(axis)
//         {
//           case 1:
// //              r11= -r11;
// //              r12= -r12;
// //              r13= -r13;
//              x= -x;
//           break;
//           case 2:
// //              r21= -r21;
// //              r22= -r22;
// //              r23= -r23;
//              y= -y;
//           break;
//           case 3:
// //              r31= -r31;
// //              r32= -r32;
// //              r33= -r33;
//              z= -z;
//           break;
//         }
        p3d_mat4Mult(T, T1, T2);
//         out << "p3d_set_prim_pos_by_mat " << str << r11 << " " << r12 << " " << r13 << " "  << x << " ";
//         out << r21 << " " << r22 << " " << r23 << " "  << y << " ";
//         out << r31 << " " << r32 << " " << r33 << " "  << z << " 0 0 0 1 " << std::endl;
        out << "p3d_set_prim_pos_by_mat " << str << T2[0][0] << " " << T2[0][1] << " " << T2[0][2] << " "  << T2[0][3] << " ";
        out << T2[1][0] << " " << T2[1][1] << " " << T2[1][2] << " "  << T2[1][3] << " ";
        out << T2[2][0] << " " << T2[2][1] << " " << T2[2][2] << " "  << T2[2][3] << " 0 0 0 1 " << std::endl;
        continue;
     }

     if(strcmp(str, "p3d_set_pos_axe")==0 )//&& !relative)
     {
//         sscanf(line, "%s %f %f %f %f %f %f", str, &x, &y, &z, &r11, &r12, &r13);
        sscanf(line, "%s %lf %lf %lf %lf %lf %lf", str, &p1[0], &p1[1], &p1[2], &r1[0], &r1[1], &r1[2]);
        p3d_xformVect(T, p1, p2);
        p3d_xformVect(T, r1, r2);
//         switch(axis)
//         {
//           case 1:
// //              r11= -r11;
//              x= -x;
//           break;
//           case 2:
// //              r12= -r12;
//              y= -y;
//           break;
//           case 3:
// //              r13= -r13;
//              z= -z;
//           break;
//         }
//         out << "p3d_set_pos_axe " << x << " " << y << " " << z << " " << r11 << " " << r12 << " " << r13 << std::endl;
        out << "p3d_set_pos_axe " << p2[0] << " " << p2[1] << " " << p2[2] << " " << r2[0] << " " << r2[1] << " " << r2[2] << std::endl;
        continue;
     }


     if(strcmp(str, "p3d_set_pos_mat")==0 )//&& !relative)
     {
//         sscanf(line, "%s %f %f %f %f %f %f %f %f %f %f %f %f", str, &r11, &r12, &r13, &x, &r21, &r22, &r23, &y, &r31, &r32, &r33, &z);
        sscanf(line, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", str, &T1[0][0], &T1[0][1], &T1[0][2], &T1[0][3], &T1[1][0], &T1[1][1], &T1[1][2], &T1[2][3], &T1[2][0], &T1[2][1], &T1[2][2], &T1[2][3]);
        p3d_mat4Mult(T, T1, T2);
//         switch(axis)
//         {
//           case 1:
// //              r11= -r11;
// //              r12= -r12;
// //              r13= -r13;
//              x= -x;
//           break;
//           case 2:
// //              r21= -r21;
// //              r22= -r22;
// //              r23= -r23;
//              y= -y;
//           break;
//           case 3:
// //              r31= -r31;
// //              r32= -r32;
// //              r33= -r33;
//              z= -z;
//           break;
//         }
//         out << "p3d_set_pos_mat " << r11 << " " << r12 << " " << r13 << " "  << x << " ";
//         out << r21 << " " << r22 << " " << r23 << " "  << y << " ";
//         out << r31 << " " << r32 << " " << r33 << " "  << z << " 0 0 0 1 " << std::endl;
        out << "p3d_set_pos_mat " << T2[0][0] << " " << T2[0][1] << " " << T2[0][2] << " "  << T2[0][3] << " ";
        out << T2[1][0] << " " << T2[1][1] << " " << T2[1][2] << " "  << T2[1][3] << " ";
        out << T2[2][0] << " " << T2[2][1] << " " << T2[2][2] << " "  << T2[2][3] << " 0 0 0 1 " << std::endl;
        continue;
     }

     if(strcmp(str, "p3d_beg_desc_jnt")==0)
     {   relative= false;    }

     if(strcmp(str, "p3d_set_pos_relative")==0)
     {   relative= true;   }

     out << line << std::endl;
  }

  return GP_OK;
}

//! WIP
int gpMirror_robot(p3d_rob *robot, int axis)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpMirror_robot(): input robot is NULL.\n", __FILE__, __LINE__);
    return GP_ERROR;
  }
  #endif

  int i;
  p3d_matrix4 T, T2, Tprev, Tprev_inv;

glPushMatrix();
glTranslatef(0.2, 0, 0);


  printf("\n\n\n\nrobot \"%s\": \n", robot->name);
  for(i=1; i<=robot->njoints; i++)
  {
    printf("\tjoint \"%s\"\n", robot->joints[i]->name);
    p3d_mat4Copy(robot->joints[i]->abs_pos, T);
    if(robot->joints[i]->prev_jnt!=NULL)
    {  
      p3d_mat4Copy(robot->joints[i]->prev_jnt->abs_pos, Tprev);
    }
    else continue;
     
   switch(axis)
   {
     case 1:
      if(i>1)
      {     
        T[1][0]= -T[1][0];
        T[1][1]= -T[1][1];
        T[1][2]= -T[1][2];
        T[2][0]= -T[2][0];
        T[2][1]= -T[2][1];
        T[2][2]= -T[2][2];
  
        T[0][3]= -T[0][3];
      }

      if(i>3)// && robot->joints[i]->prev_jnt->prev_jnt!=NULL)
      {    
        Tprev[1][0]= -Tprev[1][0];
        Tprev[1][1]= -Tprev[1][1];
        Tprev[1][2]= -Tprev[1][2];
        Tprev[2][0]= -Tprev[2][0];
        Tprev[2][1]= -Tprev[2][1];
        Tprev[2][2]= -Tprev[2][2];
  
        Tprev[0][3]= -Tprev[0][3];
      }
 
      p3d_matInvertXform(Tprev, Tprev_inv);
      p3d_mat4Mult(T, Tprev_inv, T2);
//       p3d_mat4Mult(Tprev_inv, T, T2);

      g3d_draw_frame(T,0.05);
//       p3d_mat4Print(T2, "T2");

      printf("p3d_set_pos_axe %g %g %g %g %g %g\n",T2[0][3],T2[1][3],T2[2][3],T2[0][2],T2[1][2],T2[2][2]);
      printf("p3d_set_pos_mat %g %g %g %g  %g %g %g %g  %g %g %g %g  0 0 0 1 \n",T2[0][0],T2[0][1],T2[0][2],T2[0][3],T2[1][0],T2[1][1],T2[1][2],T2[1][3],T2[2][0],T2[2][1],T2[2][2],T2[2][3]);
     break;
   }

  }

  glPopMatrix();

  return GP_ERROR;
}

//! Saves in different files all the body models of the robot after having mirroring them
//! with respect to the specified plane. The files will have the names of the bodies.
//! \param robot pointer to the robot
//! \param path the file of the folder wherein to save the files
//! \param axis bodies will be mirrored wrt this value (1= X, 2= Y, 3= Z)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpMirror_robot_bodies(p3d_rob *robot, std::string path, int axis)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpMirror_robot(): input robot is NULL.\n", __FILE__, __LINE__);
    return GP_ERROR;
  }
  #endif

  int i, j;
  unsigned int m;
  unsigned int k;
  double x, y, z;
  bool flip_face;
  p3d_vector3 normal;
  p3d_polyhedre *poly= NULL;
  size_t pos;
  FILE *file= NULL;
  std::string filename, name, name2;

  switch(axis)
  {
    case 1: normal[0]= 1; normal[1]= 0; normal[2]= 0; break;
    case 2: normal[0]= 0; normal[1]= 1; normal[2]= 0; break;
    case 3: normal[0]= 0; normal[1]= 0; normal[2]= 1; break;
  }

  for(i=0; i<robot->no; i++)
  {
    filename= path + std::string(robot->o[i]->name) + ".macro";

    file= fopen(filename.c_str(), "w");
 
    if(file==NULL)
    {
      printf("%s: %d: gpMirror_robot_bodies(): could not open file \"%s\".\n", __FILE__, __LINE__,filename.c_str());
      continue;
    }

    name= robot->o[i]->name;
    pos= name.find_last_of(".");
    name2= name.substr(pos+1);

    fprintf(file, "p3d_beg_desc P3D_BODY %s\n\n", name2.c_str());

    for(j=0; j<robot->o[i]->np; j++)
    {
      if(robot->o[i]->pol[j]->p3d_objPt!=robot->o[i]) 
      {  continue;  }

      poly= robot->o[i]->pol[j]->poly;

      fprintf(file, "\t p3d_add_desc_poly polyhedre%d \n", j+1);
      for(k=0; k<poly->nb_points; k++)
      {
        x= poly->the_points[k][0];
        y= poly->the_points[k][1];
        z= poly->the_points[k][2];

        switch(axis)
        {
          case 1: x= -x; break;
          case 2: y= -y; break;
          case 3: z= -z; break;
        }
        fprintf(file, "\t\t p3d_add_desc_vert %3.7f %3.7f %3.7f\n", x, y, z);
      }
      for(k=0; k<poly->nb_faces; k++)
      {
        if(poly->the_faces[k].plane==NULL)
        {   p3d_build_plane_face(poly, k+1);  }

        if( fabs(p3d_vectDotProd(poly->the_faces[k].plane->normale, normal)) > 1e-6 )
        {  flip_face= true; }
        else
        {  flip_face= false; }

        fprintf(file, "\t\t p3d_add_desc_face");
        if(!flip_face)
        {
          for(m=0; m<poly->the_faces[k].nb_points; m++)
          {    fprintf(file, " %d", poly->the_faces[k].the_indexs_points[m]);     }
        } 
        else
        {  
          for(m=poly->the_faces[k].nb_points-1; m>=0; m--)
          {    fprintf(file, " %d", poly->the_faces[k].the_indexs_points[m]);     } 
        }
        fprintf(file, "\n");
      }
      fprintf(file, "\t p3d_end_desc_poly\n\n");
    }
    fprintf(file, "p3d_end_desc\n");

    fclose(file);
  }


  return GP_OK;
}

