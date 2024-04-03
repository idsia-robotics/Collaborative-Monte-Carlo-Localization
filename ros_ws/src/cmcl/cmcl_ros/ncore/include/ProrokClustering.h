/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: ProrokClustering.h                                                    #
# ##############################################################################
**/


#ifndef PROROKCLUSTERING_H
#define PROROKCLUSTERING_H

#include <vector>
#include <memory>
#include "Cluster.h"
#include "DistributionData.h"
#include "Resampling.h"



class ProrokClustering
{
 	public:

 		ProrokClustering(int k, Eigen::Vector2f detectionNoise = Eigen::Vector2f(0.15, 0.15));

 		std::shared_ptr<DistributionData> Compress(const std::vector<Particle>& particles, Eigen::Vector3f trans);

		void Fuse(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);


		std::vector<Cluster> ComputeClusters(const std::vector<Particle>& particles);

 		void findHighestVarianceCluster(std::vector<Cluster>& clusters, int& index, int& dim);

		void splitClusters(std::vector<Cluster>& clusters, int& index, int& dim);

		void ReciprocalSampling(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);

		//std::vector<Cluster> ProrokClustering(std::vector<Eigen::Vector3f>& points, std::vector<float>& weights, int k);


	private:


		float o_penalty = 0.000000001;
		int o_k = 0;
		Eigen::Vector2f o_detectionNoise;
		std::shared_ptr<Resampling> o_resampler; 
};


#endif