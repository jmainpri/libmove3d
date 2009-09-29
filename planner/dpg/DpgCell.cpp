#include "../planner/dpg/proto/DpgCell.h"

using namespace std;

  DpgCell::DpgCell(){
    _id = -1;
    _valid = -1;
  }
  DpgCell::DpgCell(int id){
    _id = id;
    _valid = -1;
  }
  DpgCell::~DpgCell(){

  }
