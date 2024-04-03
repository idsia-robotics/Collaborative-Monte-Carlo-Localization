/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: DensityEstimationTree.h                                               #
# ##############################################################################
**/


#ifndef DENSITYESTIMATIONTREE_H
#define DENSITYESTIMATIONTREE_H

#include <vector>
#include <memory>
#include "DistributionData.h"
#include <mlpack/methods/det.hpp>
#include "Resampling.h"

class DensityEstimationTree
{
 	public:

 		DensityEstimationTree(int minLeafSize, int maxLeafSize);

 		std::shared_ptr<DistributionData> Compress(const std::vector<Particle>& particles, Eigen::Vector3f trans);

 		std::shared_ptr<mlpack::DTree<arma::mat>> TrainDET(const std::vector<Particle>& particles);

 		double QueryDET(std::shared_ptr<mlpack::DTree<arma::mat>> tree, Particle particle);

		void Fuse(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);


	private:


		float o_penalty = 0.000000001;
		int o_minLeafSize = 0;
		int o_maxLeafSize = 0;
		std::shared_ptr<Resampling> o_resampler; 


};


#endif