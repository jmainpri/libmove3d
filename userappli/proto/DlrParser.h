#ifndef __DLRPARSER_H__
#define __DLRPARSER_H__
#include <iostream>
#include <vector>
#include <fstream>

class DlrParser {
public:
  //Constructors and destructors
  DlrParser(char* fileName);
  virtual ~DlrParser();
  //functions
	int parse(void);
	int parse(std::string fileName);
//////////////  Move3d Function ///////////////
//////////////  Move3d Function ///////////////
  inline std::string& getFileName(void){return _fileName;}
protected:
	void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters);
	std::vector<double> parseFrame(std::string& line);
	void removeCharFromString(const std::string& src, std::string& dest, const std::string& delimiter);
	void stripSpacesAndComments(std::string& src, std::string& dest);
private:
	std::string _fileName;
//static members
public:
};

#endif
