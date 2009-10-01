#ifndef GRASP_IO_H
#define GRASP_IO_H


typedef struct gpElementParserData
{
  std::string object_name, hand_type;
  int ID, fingerID;
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
  int ID;
  double quality;
  double frame[4][4];
  std::list<gpContact> contacts;
  std::list<double> configuration;
} gpGraspParserData;


#endif

