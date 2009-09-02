#include "../userappli/proto/DlrParser.h"
#include <stdlib.h>

DlrParser::DlrParser(char* fileName){
	_fileName.assign(fileName);
}

DlrParser::~DlrParser(){
}


int DlrParser::parse(void){
	return parse(_fileName);
}

int DlrParser::parse(std::string fileName){
	std::ifstream in (fileName.c_str(), std::ifstream::in);
  if ( in.is_open() ) {
		std::string line, tmp, nextKeyword = "", keyword, lineToProcess;
		std::vector<std::string> stringVector;
		int lineNum = 0;
		while ( std::getline ( in, line, '=') ) {
			std::string::size_type i;
			lineNum++;
			keyword.clear();
			keyword.append(nextKeyword);
			if(!nextKeyword.compare("")){//firstKeword save it and loop
				stripSpacesAndComments(line, nextKeyword);
				continue;
			}else{
				i = line.find_last_of("\n");
				tmp = line.substr(i + 1, line.size() - i);
				stripSpacesAndComments(tmp, nextKeyword);
			}
			//Process current keyword
			std::string::size_type brace = line.find_first_of("[");
			if(brace == std::string::npos){//we have only quotes
				tmp = line.substr(line.find_first_of("\"") + 1, line.find_last_of("\"") - line.find_first_of("\"") - 1);
				stripSpacesAndComments(tmp, lineToProcess);
			}else{
				tmp = line.substr(line.find_first_of("[") + 1, line.find_last_of("]") - line.find_first_of("[") - 1);
				stripSpacesAndComments(tmp, lineToProcess);
			}
			std::vector<std::string> stringVector;
			tokenize(lineToProcess, stringVector, ",");//convert to array to check the order of frame
			std::vector<double> doubleVector = parseFrame(lineToProcess);
			
			if(!keyword.compare("start_configuration")){
				std::cout << "start_configuration" << std::endl;
				//save the configuration "doubleVector"
			}else if(!keyword.compare("object")){
				std::cout << "object" << std::endl;
				std::string objectName = "";
				if(brace == std::string::npos){//it's a non manipulated object declaration find the se
					objectName.append(lineToProcess);
				}else{
					std::vector<double> leftFrame, rightFrame;
					objectName.append(stringVector[0]);
					for(int j = 2; j < 14; j++){
						if(stringVector[1].compare("left")){
							leftFrame.push_back(doubleVector[j]);
						}else{
							rightFrame.push_back(doubleVector[j]);
						}
					}
					for(int j = 15; j < 27; j++){
						if(stringVector[14].compare("left")){
							leftFrame.push_back(doubleVector[j]);
						}else{
							rightFrame.push_back(doubleVector[j]);
						}
					}
					//add the graspFrames to the object
				}
				//set the object active
			}else if(!keyword.compare("object_pose")){
				std::cout << "object_pose" << std::endl;
				std::vector<double> objectPos;
				for(int j = 2; j < 14; j++){
					objectPos.push_back(doubleVector[j]);
				}
				double objectPosId = doubleVector[1];
				std::string objectName = "";
				objectName.append(stringVector[0]);
			}else if(!keyword.compare("plan_type")){
				std::cout << "plan_type" << std::endl;
			}else if(!keyword.compare("plan_object")){
				std::cout << "plan_object" << std::endl;
			}else if(!keyword.compare("plan_object_target")){
				std::cout << "plan_object_target" << std::endl;
			}else if(!keyword.compare("plan_obstacles")){
				std::cout << "plan_obstacles" << std::endl;
			}else if(!keyword.compare("plan_execute")){
				std::cout << "plan_execute" << std::endl;
			}else{
				std::cout << "Unknown keyword at line : " << lineNum << std::endl;
			}
		}
  }
	return false;
}

void DlrParser::tokenize(const std::string& str,
												 std::vector<std::string>& tokens,
												 const std::string& delimiters = " "){
	// Skip delimiters at beginning.
	std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// Find first "non-delimiter".
	std::string::size_type pos = str.find_first_of(delimiters, lastPos);
	
	while (std::string::npos != pos || std::string::npos != lastPos)
	{
		// Found a token, add it to the vector.
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);
	}
}

std::vector<double> DlrParser::parseFrame(std::string& line){
	std::vector<double> frame;
	std::vector<std::string> stringVector;
	tokenize(line, stringVector, ",");
	for(unsigned int i = 0; i < stringVector.size(); i++){
		frame.push_back(atof(stringVector[i].c_str()));
	}
	return frame;
}

void DlrParser::removeCharFromString(const std::string& src, std::string& dest, const std::string& delimiter = " "){
	dest.clear();
	//Spaces
	// Skip delimiters at beginning.
	std::string::size_type lastPos = src.find_first_not_of(delimiter, 0);
	// Find first "non-delimiter".
	std::string::size_type pos = src.find_first_of(delimiter, lastPos);
	while (std::string::npos != pos || std::string::npos != lastPos)
	{
		// Found a token, add it to the vector.
		dest += src.substr(lastPos, pos - lastPos);
		// Skip delimiters.  Note the "not_of"
		lastPos = src.find_first_not_of(delimiter, pos);
		// Find next "non-delimiter"
		pos = src.find_first_of(delimiter, lastPos);
	}
}

void DlrParser::stripSpacesAndComments(std::string& src, std::string& dest){
	std::string tmp;
	//Remove spaces
	removeCharFromString(src, tmp, " ");
	
	//Comments
	std::string::size_type lastPos = 0;
	std::string::size_type pos = tmp.find_first_of("#", lastPos);
	lastPos = tmp.find_first_of("\n", pos);
	while( pos != std::string::npos || lastPos != std::string::npos){
		tmp.erase(pos, lastPos - pos);
		pos = tmp.find_first_of("#", pos);
		lastPos = tmp.find_first_of("\n", pos);
	}
	
	//Retruns
	removeCharFromString(tmp, dest, "\n");
	//remove quotes
	tmp.clear();
	tmp.append(dest);
	dest.clear();
	removeCharFromString(tmp, dest, "\"");
	
}
