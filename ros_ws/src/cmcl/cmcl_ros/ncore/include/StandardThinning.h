/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: StandardThinning.h          			                   		       #
# ##############################################################################
**/


#ifndef STANDARDTHINNING_H
#define STANDARDTHINNING_H


#include "DistributionData.h"
#include "Resampling.h"

class StandardThinning
{
	public: 
		StandardThinning(int subSize, Eigen::Vector2f detectionNoise = Eigen::Vector2f(0.15, 0.15));

		std::shared_ptr<DistributionData> Compress(const std::vector<Particle>& particles, Eigen::Vector3f trans);

		void Fuse(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);

		void ReciprocalSampling(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);



	private:
		float o_penalty = 0.000000001;
		Eigen::Vector2f o_detectionNoise;
		int o_n = 0;
		std::shared_ptr<Resampling> o_resampler; 


};


#endif