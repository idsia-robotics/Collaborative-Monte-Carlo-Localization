
/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: ClusterAbstraction.h                                                  #
# ##############################################################################
**/


#ifndef CLUSTERABSTRACTION_H
#define CLUSTERABSTRACTION_H

#include <vector>
#include <memory>
#include <math.h>
#include "Cluster.h"
#include "Utils.h"



class ClusterAbstraction
{
public:	

	ClusterAbstraction();

	ClusterAbstraction(Cluster& cluster, Eigen::Vector3f trans);


	static Eigen::Vector2f computePolarDifference(Eigen::Vector3f pm, Eigen::Vector3f pn);

	static std::vector<Eigen::Vector3f> shiftPoints(std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& trans);

	static Eigen::Vector3f shiftPoint(Eigen::Vector3f& point, const Eigen::Vector2f& trans);

	void computePolarVariance(std::vector<Eigen::Vector2f>& polarPoints, std::vector<float>& weights);




	

	Eigen::Vector3f centroid;
	float weight;
	Eigen::Vector2f mu;
	Eigen::Matrix2f covariance;
};

#endif