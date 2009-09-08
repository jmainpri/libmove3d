#ifndef __DLRPARSER_H__
#define __DLRPARSER_H__
#include <iostream>
#include <vector>
#include <fstream>
#include "../userappli/proto/DlrPlanner.h"

class DlrParser {
public:
  //Constructors and destructors
  DlrParser(char* fileName);
	DlrParser(char* fileName, DlrPlanner* planner);
  virtual ~DlrParser();
  //functions
	int parse(void);
	int parse(std::string fileName);
//////////////  Move3d Function ///////////////
//////////////  Move3d Function ///////////////
  inline std::string& getFileName(void){return _fileName;}
  inline void setPlanner(DlrPlanner* planner){_planner = planner;}
  inline DlrPlanner* getPlanner(){return _planner;}
protected:
	void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters);
	std::vector<double> parseFrame(std::string& line);
	void removeCharFromString(const std::string& src, std::string& dest, const std::string& delimiter);
	void stripSpacesAndComments(std::string& src, std::string& dest);
private:
	std::string _fileName;
  DlrPlanner* _planner;
//static members
public:
};

#endif
