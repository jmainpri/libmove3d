

extern int gpSave_grasp_list(std::list<gpGrasp> &graspList, std::string filename);

extern std::string getNodeString(xmlDocPtr doc, xmlNodePtr node);

extern void warningMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message);

extern void formatErrorMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message);

extern void elementMissingMessage(int line_number, const xmlChar *URL, const xmlChar *element_name, std::string &message);

extern bool gpParseElement(xmlDocPtr doc, xmlNodePtr entry_node, std::string element, gpElementParserData &data);

extern bool gpParseContact(xmlDocPtr doc, xmlNodePtr entry_node, gpGraspParserData &data);

extern bool gpParseGrasp(xmlDocPtr doc, xmlNodePtr entry_node, gpGraspParserData &data);

extern int gpParseGraspListFile(std::string filename, std::list<gpGrasp> &graspList);
