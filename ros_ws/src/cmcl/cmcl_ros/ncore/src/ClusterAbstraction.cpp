

/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: ClusterAbstraction.cpp                                                #
# ##############################################################################
**/

#include "ClusterAbstraction.h"
#include <functional>
#include <iostream>
#include <numeric>

std::vector<Eigen::Vector3f> ClusterAbstraction::shiftPoints(std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& trans)
{
	size_t numPoints = points.size();

	float r = trans.head(2).norm();
	float theta = atan2(trans(1), trans(0));

	std::vector<Eigen::Vector3f> shiftedPoints(numPoints);

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < numPoints; ++i)
	{
			Eigen::Vector3f p;
			p(0) = points[i](0) + r * cos(points[i](2) + theta);
			p(1) = points[i](1) + r * sin(points[i](2) + theta);
			shiftedPoints[i] = p;
	}

	return shiftedPoints;	
}

Eigen::Vector3f ClusterAbstraction::shiftPoint(Eigen::Vector3f& point, const Eigen::Vector2f& trans)
{
	float r = trans(0);
	float theta = trans(1);

	Eigen::Vector3f p;
	p(0) = point(0) + r * cos(point(2) + theta);
	p(1) = point(1) + r * sin(point(2) + theta);
	p(2) =  point(2);


	return p;
}


Eigen::Vector2f ClusterAbstraction::computePolarDifference(Eigen::Vector3f pm, Eigen::Vector3f pn)
{
	Eigen::Vector2f diff(0, 0);

	diff(0) = (pm.head(2) - pn.head(2)).norm();
	diff(1) = Wrap2Pi(atan2(pn(1) - pm(1), pn(0) - pm(0)) - (pm(2))); //Do I need to add M_PI here?

	return diff;
}


void ClusterAbstraction::computePolarVariance(std::vector<Eigen::Vector2f>& polarPoints, std::vector<float>& weights)
{
	int pointNum = polarPoints.size();
	double dx = 0;
	double dy = 0;
	double w = 0;
	double r = 0;

	mu = Eigen::Vector2f(0, 0);
	covariance = Eigen::Matrix2f::Zero();

	if (pointNum == 0)
	{
		mu = Eigen::Vector2f(NAN, NAN);	
		return;
	}

	for (int i = 0; i < pointNum; ++i)
	{
		r += polarPoints[i](0) * weights[i];
		dx += cos(polarPoints[i](1)) * weights[i]; // theta 
		dy += sin(polarPoints[i](1)) * weights[i]; // theta
		w +=  weights[i];
	}

	mu(0) = r / w;
	mu(1) = Wrap2Pi(atan2(dy, dx));

	if (pointNum > 1)
	{
		for (int i = 0; i < pointNum; ++i)
		{
			Eigen::Vector2f diff = polarPoints[i] - mu;
			//covariance += diff * diff.transpose();
			covariance(0,0) += diff(0) * diff(0) * weights[i];
			covariance(1,1) += diff(1) * diff(1) * weights[i];
		}

		covariance /= w;
	}

}


ClusterAbstraction::ClusterAbstraction(Cluster& cluster, Eigen::Vector3f trans)
{
	int numPoints = cluster.points.size();
	centroid = cluster.mean;
	weight = std::accumulate(cluster.weights.begin(), cluster.weights.end(), 0.0);

	std::vector<Eigen::Vector3f> shiftedPoints = ClusterAbstraction::shiftPoints(cluster.points, trans);
	std::vector<Eigen::Vector2f> polarDiff(numPoints);

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < numPoints; ++i)
	{
		polarDiff[i] = ClusterAbstraction::computePolarDifference(centroid, shiftedPoints[i]);
	}

	computePolarVariance(polarDiff, cluster.weights);
}

ClusterAbstraction::ClusterAbstraction() 
{

	centroid = Eigen::Vector3f();
	weight = 0;
	mu = Eigen::Vector2f();
	covariance = Eigen::Matrix2f::Zero();
}

