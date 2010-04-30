#ifndef HRICS_VISIBILITY_H
#define HRICS_VISIBILITY_H
/*
 *  HRICS_Visbility.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 30/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

/**
 @ingroup HRICS
 */
namespace HRICS
{
    class Visibility 
	{
	public:
		
		double getVisibilityCost(Vector3d WSPoint);
		
	};
}

#endif
