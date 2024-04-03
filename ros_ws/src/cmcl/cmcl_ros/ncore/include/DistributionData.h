/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: DistributionData.h          			                   		       #
# ##############################################################################
**/


#ifndef DISTRIBUTIONDATA_H
#define DISTRIBUTIONDATA_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include "Particle.h"
#include "ClusterAbstraction.h"


enum class DISTRIBUTION
{   
	PARTICLES = 0,
	DET = 1, 
    KMEANS = 2,
    PROROK = 3,
    GOODPOINTS = 4
};

class DistributionData 
{

	public: 

		DistributionData(DISTRIBUTION distType, std::vector<Particle> observerParticles, Eigen::Vector2f variance = Eigen::Vector2f(1.0, 1.0))
		{
			o_distType = distType;
			o_particles = observerParticles;
			o_variance = variance;
		}

		DistributionData(DISTRIBUTION distType, std::vector<uint8_t> det, Eigen::Vector2f variance = Eigen::Vector2f(1.0, 1.0))
		{
			o_distType = distType;
			o_det = det;
			o_variance = variance;
		}

		DistributionData(DISTRIBUTION distType, std::vector<Cluster> kmeans)
		{
			o_distType = distType;
			o_kmeans = kmeans;
			o_variance = Eigen::Vector2f(0.0, 0.0);
		}

		DistributionData(DISTRIBUTION distType, std::vector<ClusterAbstraction> prorok)
		{
			o_distType = distType;
			o_prorok = prorok;
			o_variance = Eigen::Vector2f(0.0, 0.0);
		}

		std::vector<Particle>& Particles()
		{
			return o_particles;
		}

		std::vector<uint8_t>& DET()
		{
			return o_det;
		}

		std::vector<Cluster>& KMeans()
		{
			return o_kmeans;
		}

		std::vector<ClusterAbstraction>& Prorok()
		{
			return o_prorok;
		}

		DISTRIBUTION Type()
		{
			return o_distType;
		}

		Eigen::Vector2f Variance()
		{
			return o_variance;
		}

		

	private:

		DISTRIBUTION o_distType;
		std::vector<Particle> o_particles;
		std::vector<uint8_t> o_det;
		std::vector<ClusterAbstraction> o_prorok;
		std::vector<Cluster> o_kmeans;
		Eigen::Vector2f o_variance;

};

#endif