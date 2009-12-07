#ifndef HRIDISTANCE_H
#define HRIDISTANCE_H

/*
 *  HRICS_Distance.h
 *  
 *
 *  Created by Jim Mainprice on 05/12/09.
 *  Copyright 2009 mainprice@gmail.com All rights reserved.
 *
 */

#include "../API/planningAPI.hpp"

class HRICS_Distance {

public:
	HRICS_Distance();
        HRICS_Distance(Robot* rob, std::vector<Robot*> humans);

        ~HRICS_Distance();
	
	void drawZones();
	void parseHumans();
	double getDistance();
        void offSetPrim(p3d_poly* poly,double offset);

        void activateSafetyZonesMode();
        void activateNormalMode();

        std::vector<double> getDistToZones();
        std::vector<double> getVectorJim() {return vect_jim; }

	
private:
	Robot* _Robot;
	std::vector<Robot*> _Humans;
        std::vector< std::vector<int> > _SafetyZonesBodyId;
        std::vector<double> _PenetrationDist;
        std::vector<double> vect_jim;
        double _SafeOffset;
        double _SafeRadius;
};

#endif
