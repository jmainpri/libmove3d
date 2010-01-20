#ifndef HRIDISTANCE_H
#define HRIDISTANCE_H

/*
 *  Distance.h
 *  
 *
 *  Created by Jim Mainprice on 05/12/09.
 *  Copyright 2009 mainprice@gmail.com All rights reserved.
 *
 */

#include "../API/planningAPI.hpp"

/**
  @ingroup HRICS
  */
namespace HRICS
{
    class Distance {

    public:
        Distance();
        Distance(Robot* rob, std::vector<Robot*> humans);

        ~Distance();

        void drawZones();
        void parseHumans();
        double getDistance();
        void offSetPrim(p3d_poly* poly,double offset);

        void activateSafetyZonesMode();
        void activateNormalMode();

        std::vector<double> getDistToZones();
        std::vector<double> getVectorJim() {return vect_jim; }

        void setVector( std::vector<double> toDrawVector ) { vect_jim = toDrawVector; }

        double computeBBDist(p3d_vector3 robot, p3d_vector3 human);
        double computeBoundingBalls(p3d_vector3 robot, p3d_vector3 human);


    private:
        Robot* _Robot;
        std::vector<Robot*> _Humans;
        std::vector< std::vector<int> > _SafetyZonesBodyId;
        std::vector<double> _PenetrationDist;
        std::vector<double> vect_jim;
        double _SafeOffset;
        double _SafeRadius;
    };
}

#endif
