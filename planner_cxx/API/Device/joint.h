/*
 *  joint.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 12/05/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef JOINT_HPP
#define JOINT_HPP

/*!
 @ingroup ROBOT
 @brief This class holds a Joint and is associated with a Body (Link)
 It's the basic element of a kinematic chain
 */
class Joint {
	
public:
	/**
     * Constructor
     * @param The p3d_jnt that is used
     */
    Joint(p3d_jnt* R , bool copy = false );
	
    /**
     * Destructor of the class
     */
    ~Joint();
	
	/**
	 * Get the Matrix abs_pos of the Joint 
	 */
	Eigen::Transform3d	getMatrixPos();
	
	/**
	 * Get the Vector abs_pos of the Joint 
	 */
	Eigen::Vector3d		getVectorPos();
	
	
private:
	p3d_jnt* m_Joint; /*!< The p3d structure for the Joint*/
    std::string m_Name; /*!< The Joint's Name */
	bool m_copy; /*!< Is true if the p3d_jnt copies and not only points to the structure */
	
};

#endif