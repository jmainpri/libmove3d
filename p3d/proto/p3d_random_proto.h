/*
 *  p3d_random_proto.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 03/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

extern void p3d_init_random_seed(int seed);
extern uint p3d_init_random(void);
extern double p3d_random(double a, double b);
int p3d_random_integer(int a, int b);
extern double NormalRand( double Sigma_d_a );
extern double p3d_gaussian_random( const double& mean, const double& variance , const double& min, const double& max);
extern double p3d_gaussian_random2( const double& mean, const double& variance );
