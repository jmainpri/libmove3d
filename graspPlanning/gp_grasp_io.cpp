
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "UserAppli-pkg.h"
#include "GraspPlanning-pkg.h"
#include <math.h>
#include <string>
#include <sstream>

//! Saves a grasp list as a file in XML format.
//! \param graspList the grasp list to save
//! \param filename the file where to save the list in (if it has no .xml extension, the extension will be added)
//! \return 1 in case of success, 0 otherwise
int gpSave_grasp_list(std::list<gpGrasp> &graspList, std::string filename)
{
  unsigned int i;
  size_t pos;
  gpHand_type hand_type;
  std::string object_name;
  FILE *file= NULL;
  std::list<gpGrasp>::iterator grasp;

  if(graspList.empty())
  {
    printf("%s: %d: gpSave_grasp_list(): the grasp list is empty.\n",__FILE__,__LINE__);
    return 0;
  }
  object_name= graspList.front().object_name;
  hand_type= graspList.front().hand_type;

  pos= filename.find(".xml"); 
  if(pos!=filename.length()-4)
  {
    filename+= ".xml";
  }    

  file= fopen(filename.c_str(),"w");

  fprintf(file, "<!--grasp list for object \"%s\" with \"%s\" hand --> \n", object_name.c_str(), (gpHand_type_to_string(hand_type)).c_str());

  fprintf(file, "<grasp_list nb_elements=\"%d\"> \n", graspList.size());
  fprintf(file, "  <object_name> %s </object_name> \n", object_name.c_str());
  fprintf(file, "  <hand_type> %s </hand_type> \n", (gpHand_type_to_string(hand_type)).c_str());

  for(grasp=graspList.begin(); grasp!=graspList.end(); grasp++)
  {
    fprintf(file, "  <grasp ID=\"%d\" nb_contacts=\"%d\" nb_dofs=\"%d\"> \n", grasp->ID, grasp->contacts.size(), grasp->config.size());
    fprintf(file, "    <ID> %d </ID>\n", grasp->ID);
    fprintf(file, "    <object_name> %s </object_name> \n", grasp->object_name.c_str());
    fprintf(file, "    <hand_type> %s </hand_type> \n", gpHand_type_to_string(grasp->hand_type).c_str());
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
      fprintf(file, "      <fingerID> %d </fingerID> \n", grasp->contacts[i].fingerID);
      fprintf(file, "      <friction_coefficient> %f </friction_coefficient> \n", grasp->contacts[i].mu);
      fprintf(file, "    </contact> \n");
    }

    fprintf(file, "    <configuration> \n");
    for(i=0; i<grasp->config.size(); i++)
    {
      fprintf(file, "  %f\n", grasp->config[i]);
    }
    fprintf(file, "    </configuration> \n");

    fprintf(file, "  </grasp> \n");
  }
  fprintf(file, "</grasp_list> \n");

  fclose(file);

  return 1;
}


//! Converts the char string of the given node to a std::string after removing all newline characters
//! as well as space characters at the end of the string.
std::string getNodeString(xmlDocPtr doc, xmlNodePtr node)
{
  size_t pos;
  xmlChar *key;
  std::string text, message, word;
  key= xmlNodeListGetString(doc, node->xmlChildrenNode, 1);
  text= (char *) key;
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



void warningMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message)
{
   printf("At line %d in %s: warning: in an element of <%s>: \n", line_number, URL, element_name);
   printf(" %s\n\n", message.c_str());
   return;
}


void formatErrorMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message)
{
   printf("At line %d in %s: format error in the data of an element <%s>.\n", line_number, URL, element_name);
   printf(" %s\n\n", message.c_str());

   return;
}

void elementMissingMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message)
{
   printf("At line %d in %s: A sub-element of element <%s> is missing.\n", line_number, URL, element_name);
   printf(" %s\n\n", message.c_str());

   return;
}


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
  xmlNodePtr cur= entry_node->xmlChildrenNode;
  std::string text, message, word;
  std::istringstream iss;

  for(cur= entry_node->xmlChildrenNode; cur!=NULL; cur= cur->next)
  {
    if( !xmlStrcmp(cur->name, (xmlChar*) element.c_str() ) )
    {
       iss.str(getNodeString(doc, cur));

       if(element=="object_name")
       { 
         result= (iss >> data.object_name );
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
         data.frame[3][0] >> data.frame[3][1] >> data.frame[3][2] >> data.frame[3][3]  );

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

       if(element=="quality")
       {
         result= (iss >> data.quality );
         if( !result || !iss.eof() )
         {
           message= "Usage: <quality> grasp quality </quality>.";
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

       if(element=="configuration")
       {
          data.configuration.clear();
          while(!iss.eof())
          {
            result= (iss >> x );
            if(!result)
            {
              message= "Usage: <configuration> hand's joint parameters (angles are given in radians, lengths in meters) </q>.";
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

    }   
  }

  return false;
}



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

  return true;
}


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
  {printf(">>%s<<\n", elementData.hand_type.c_str());
    if(elementData.hand_type=="GP_GRIPPER")
    {  data.hand_type= GP_GRIPPER;  }
    else if(elementData.hand_type=="GP_SAHAND_RIGHT")
    {  data.hand_type= GP_SAHAND_RIGHT;  }
    else if(elementData.hand_type=="GP_SAHAND_LEFT")
    {  data.hand_type= GP_SAHAND_LEFT;    }
    else
    {
      message= "Usage: <hand_type> type of the robot gripper or hand (GP_GRIPPER, GP_SAHAND_RIGHT or GP_SAHAND_LEFT) </hand_type>.";
      formatErrorMessage((int) xmlGetLineNo(entry_node), doc->URL, entry_node->name, message);
      return false;
    }
  }

  if(gpParseElement(doc, entry_node, "ID", elementData)) 
  {
    data.ID= elementData.ID;
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


  // Read all the contact elements:
  data.contacts.clear();
  for(cur= entry_node->xmlChildrenNode; cur!=NULL; cur= cur->next)
  {
    if((!xmlStrcmp(cur->name, (const xmlChar *)"contact")))
    { 
      if(gpParseContact(doc, cur, contactData))
      { 
         contact.fingerID= contactData.fingerID;
         p3d_vectCopy(contactData.position, contact.position);
         p3d_vectCopy(contactData.normal, contact.normal);
         contact.mu= contactData.friction_coefficient;
         data.contacts.push_back(contact);
      }
    }
  }
  

  return true;
}


//! Parses an XML file describing a grasp list.
//! \param filename name of the XML file
//! \param graspList the grasp list that will be filled with the content of the file
//! \return 1 in case of success, 0 otherwise
int gpParseGraspListFile(std::string filename, std::list<gpGrasp> &graspList)
{
  xmlDocPtr doc;
  xmlNodePtr root, cur;
  std::string text, message;  
  gpElementParserData elementData;
  gpGraspParserData graspData;
  gpGrasp grasp;
  std::list<gpContact>::iterator contact_iter;
  std::list<double>::iterator q_iter;

  xmlLineNumbersDefault(1);
  doc= xmlParseFile(filename.c_str());

  if(doc==NULL)
  {
    fprintf(stderr, "Document \"%s\" was not parsed successfully.\n", filename.c_str());
    return 0;
  }
	
  root = xmlDocGetRootElement(doc);
	
  if(root==NULL)
  {
    fprintf(stderr, "Document \"%s\" is empty.\n", filename.c_str());
    xmlFreeDoc(doc);
    return 0;
  }
	
  if(xmlStrcmp(root->name, (const xmlChar *) "grasp_list"))
  {
    fprintf(stderr, "Document \"%s\" is of the wrong type, root node != <grasp_list>.\n", filename.c_str());
    xmlFreeDoc(doc);
    return 0;
  }
	
  //Read all the grasp elements:
  for(cur= root->xmlChildrenNode; cur!=NULL; cur= cur->next)
  {
    if((!xmlStrcmp(cur->name, (const xmlChar *)"grasp")))
    { 
      if(gpParseGrasp(doc, cur, graspData))
      { 
         grasp.object_name= graspData.object_name;
         grasp.hand_type= graspData.hand_type;
         grasp.ID= graspData.ID;
         grasp.quality= graspData.quality;
         p3d_mat4Copy(graspData.frame, grasp.frame);
         grasp.contacts.clear();
         grasp.contacts.reserve(graspData.contacts.size());
         for(contact_iter=graspData.contacts.begin(); contact_iter!=graspData.contacts.end(); contact_iter++)
         {
           grasp.contacts.push_back(*contact_iter);
         }
         grasp.config.clear();
         grasp.config.reserve(graspData.configuration.size());
         for(q_iter=graspData.configuration.begin(); q_iter!=graspData.configuration.end(); q_iter++)
         {
           grasp.config.push_back(*q_iter);
         }
         graspList.push_back(grasp);
      }
    }
  }
  

  xmlFreeDoc(doc);
  return 1;
}


