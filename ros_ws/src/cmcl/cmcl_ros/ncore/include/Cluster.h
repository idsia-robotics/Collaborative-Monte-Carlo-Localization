/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: Cluster.h                                                             #
# ##############################################################################
**/

#ifndef CLUSTER_H
#define CLUSTER_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>

class Cluster
{
public:

	Cluster(){}

	void ComputeMean();
	
	void ComputeVariance();

	void ComputeError();

	void ComputeWeight();
	

	std::vector<Eigen::Vector3f> points;
	std::vector<float> weights;
	float totalWeight = 0;
	Eigen::Vector3f mean;
	Eigen::Vector3f variance;
	float error;


};



#endif