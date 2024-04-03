/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: DistributionExchange.h    				                               #
# ##############################################################################
**/

#ifndef DISTRIBUTIONEXCHANGE_H
#define DISTRIBUTIONEXCHANGE_H

#include <memory>
#include "Particle.h"
#include "DistributionData.h"

#include "GoodPointsCompression.h"
#include "ProrokClustering.h"
#include "DensityEstimationTree.h"
#include "KMeansClustering.h"
#include "StandardThinning.h"


class DistributionExchange
{
	public:

		DistributionExchange(const std::string& jsonConfigPath);

		DistributionExchange(int k = 5, Eigen::Vector2f detectionNoise = Eigen::Vector2f(0.15, 0.15), float penalty = 0.000000001);

		void FuseDistribution(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);

		std::shared_ptr<DistributionData> CreateDistribution(const std::vector<Particle>& observerParticles, DISTRIBUTION distType, Eigen::Vector3f trans = Eigen::Vector3f());

		


	private:

		Eigen::Vector2f o_detectionNoise;
		float o_penalty = 0.000000001;
		int o_k = 5;

		std::shared_ptr<GoodPointsCompression> o_goodPoints;
		std::shared_ptr<ProrokClustering> o_prorok;
		std::shared_ptr<DensityEstimationTree> o_det;
		std::shared_ptr<KMeansClustering> o_kmeans;
		std::shared_ptr<StandardThinning> o_thin;
		//std::shared_ptr<StandardThinning> o_naive;

};





#endif
