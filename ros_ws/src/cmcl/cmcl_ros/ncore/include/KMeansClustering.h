/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: KMeansClustering.h                                                    #
# ##############################################################################
**/


#ifndef KMEANSCLUSTERING_H
#define KMEANSCLUSTERING_H

#include <vector>
#include <memory>
#include "DistributionData.h"
#include "Resampling.h"

class KMeansClustering
{
public:

	KMeansClustering(int k, int epochs, Eigen::Vector2f detectionNoise);

	std::shared_ptr<DistributionData> Compress(const std::vector<Particle>& particles, Eigen::Vector3f trans);

	void Fuse(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);

	void AssignClusters(std::vector<Eigen::Vector3f>& points,  std::vector<float>& weights, std::vector<Cluster>& clusters);

	void RecomputeCentroids(std::vector<Cluster>& clusters);

	void ReciprocalSampling(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);


private:

	float o_penalty = 0.000000001;
	int o_k = 0;
	int o_epochs = 0;
	Eigen::Vector2f o_detectionNoise;
	std::shared_ptr<Resampling> o_resampler; 
};

#endif