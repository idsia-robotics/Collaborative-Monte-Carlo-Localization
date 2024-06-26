/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: Particle.h           				                           		   #
# ##############################################################################
**/

#ifndef PARTICLE_H
#define PARTICLE_H


#include <vector>
#include <eigen3/Eigen/Dense>


class Particle
{
public:

	Particle(Eigen::Vector3f p = Eigen::Vector3f(0, 0, 0), double w = 0);
	
	Eigen::Vector3f pose;
	double weight;

};


#endif