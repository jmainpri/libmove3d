#ifndef GRASP_IO_H
#define GRASP_IO_H


typedef struct gpElementParserData
{
  std::string object_name, hand_type;
  int ID, fingerID, body_index;
  double quality, friction_coefficient;
  double position[3], normal[3];
  double frame[4][4];
  std::list<double> configuration;
} gpElementParserData;


typedef struct gpContactParserData
{
  double friction_coefficient;
  double position[3], normal[3];
  int fingerID;
} gpContactParserData;


typedef struct gpGraspParserData
{
  std::string object_name;
  gpHand_type hand_type;
  int body_index;
  int ID;
  double quality;
  double frame[4][4];
  std::list<gpContact> contacts;
  std::list<double> configuration;
} gpGraspParserData;

extern int gpSave_grasp_list(std::list<gpGrasp> &graspList, std::string filename);

extern std::string getNodeString(xmlDocPtr doc, xmlNodePtr node);

extern void warningMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message);

extern void formatErrorMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message);

extern void elementMissingMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message);

extern bool gpParseElement(xmlDocPtr doc, xmlNodePtr entry_node, std::string element, gpElementParserData &data);

extern bool gpParseContact(xmlDocPtr doc, xmlNodePtr entry_node, gpGraspParserData &data);

extern bool gpParseGrasp(xmlDocPtr doc, xmlNodePtr entry_node, gpGraspParserData &data);

extern int gpLoad_grasp_list(std::string filename, std::list<gpGrasp> &graspList);

extern int gpInvert_axis(std::string inputFile, std::string outputFile, p3d_matrix4 T);

extern int gpMirror_robot(p3d_rob *robot, int axis);

extern int gpMirror_robot_bodies(p3d_rob *robot, std::string path, int axis);

#endif

