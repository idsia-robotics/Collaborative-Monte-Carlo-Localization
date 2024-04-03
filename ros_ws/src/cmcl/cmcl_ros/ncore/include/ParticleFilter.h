/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: ParticleFilter.h             		                           		   #
# ##############################################################################
**/
 

#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include "Particle.h"
#include "GMap.h"
#include "SetStatistics.h"

class ParticleFilter
{
public:

	//multi-map 
	ParticleFilter(std::shared_ptr<GMap> GMap);

	

	void InitUniform(std::vector<Particle>& particles, int n_particles);
	void InitGaussian(std::vector<Particle>& particles, int n_particles, const std::vector<Eigen::Vector3f>& initGuess, const std::vector<Eigen::Matrix3d>& covariances);
   
	SetStatistics ComputeStatistics(const std::vector<Particle>& particles);

	void NormalizeWeights(std::vector<Particle>& particles);

	
	std::vector<Particle>& Particles()
	{
		return o_particles;
	}

	void SetParticle(int id, Particle p)
	{
		o_particles[id] = p;
	}


	SetStatistics Statistics()
	{
		return o_stats;
	}



private:



	
	std::shared_ptr<GMap> o_gmap;
	std::vector<Particle> o_particles;
	SetStatistics o_stats;
};

#endif