/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: Cluster.cpp                                                           #
# ##############################################################################
**/

#include "Cluster.h"
#include "Utils.h"

#include <iostream>

void Cluster::ComputeMean()
{
	int pointNum = points.size();
	mean = Eigen::Vector3f(0,0,0);

	double dx = 0;
	double dy = 0;
	double w = 0;

	for (int i = 0; i < pointNum; ++i)
	{
		mean(0) += points[i](0) * weights[i];
		mean(1) += points[i](1) * weights[i];
		dx += cos(points[i](2)) * weights[i]; // theta 
		dy += sin(points[i](2)) * weights[i]; // theta
		w += weights[i];
	}

	mean /= w;
	mean(2) = Wrap2Pi(atan2(dy, dx));

	//std::cout << "dx dy:" << dx << ", " << dy << ", " << mean(2) << std::endl;
	//std::cout << "mean:" << mean(0) << ", " << mean(1) << ", " << mean(2) << ", " << w << std::endl;
}

void Cluster::ComputeError()
{
	int n = points.size();
	float error = 0;

	for (int i = 0; i < n; ++i)
	{
		Eigen::Vector2f diff = points[i].head(2) - mean.head(2);
		float dist = diff.norm();
		error += dist;
	}

	error /= n;
}

void Cluster::ComputeWeight()
{
	int n = points.size();
	totalWeight = 0;
	for (int i = 0; i < n; ++i)
	{
		totalWeight += weights[i];
	}
}

void Cluster::ComputeVariance()
{
	int pointNum = points.size();
	variance = Eigen::Vector3f(0, 0, 0);
	ComputeMean();

	double w = 0;

	for (int i = 0; i < pointNum; ++i)
	{
		Eigen::Vector3f p = points[i];
		Eigen::Vector3f diff = p - mean;
		w += weights[i];

		variance(0) += diff(0) * diff(0) * weights[i];
		variance(1) += diff(1) * diff(1) * weights[i];
		// ignoring angular variance here
	}

	variance /= w;
}