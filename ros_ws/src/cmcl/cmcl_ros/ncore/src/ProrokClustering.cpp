/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: ProrokClustering.cpp                                                  #
# ##############################################################################
**/

#include "ProrokClustering.h"
#include <iostream>


ProrokClustering::ProrokClustering(int k, Eigen::Vector2f detectionNoise)
{
	o_k = k;
	o_detectionNoise = detectionNoise;
	o_resampler = std::make_shared<Resampling>(Resampling());

}

void ProrokClustering::findHighestVarianceCluster(std::vector<Cluster>& clusters, int& index, int& dim)
{
	int k = clusters.size();

	float max_variance = 0;

	for(int i = 0; i < k; ++i)
	{
		clusters[i].ComputeVariance();

		if (clusters[i].variance(0) > clusters[i].variance(1) && clusters[i].variance(0) > max_variance)
		{
			index = i;
			max_variance = clusters[i].variance(0);
			dim = 0;
		}
		else if (clusters[i].variance(1) >= clusters[i].variance(0) && clusters[i].variance(1) > max_variance)
		{
			index = i;
			max_variance = clusters[i].variance(1);
			dim = 1;
		}
	}
}

void ProrokClustering::splitClusters(std::vector<Cluster>& clusters, int& index, int& dim)
{
	std::vector<Cluster> new_clusters;
	for(int i = 0; i < clusters.size(); ++i)
	{
		if (i != index) new_clusters.push_back(clusters[i]);
	}

	//split along the mean on the dim
	Cluster c1;
	Cluster c2;
	int numPoints = clusters[index].points.size();

	for(int i = 0; i < numPoints; ++i)
	{
		if (clusters[index].mean(dim) >= clusters[index].points[i](dim))
		{
			c1.points.push_back(clusters[index].points[i]);
			c1.weights.push_back(clusters[index].weights[i]);
		}
		else
		{
			c2.points.push_back(clusters[index].points[i]);
			c2.weights.push_back(clusters[index].weights[i]);
		}
	}


	// add new clusters to the list
	new_clusters.push_back(c1);
	new_clusters.push_back(c2);


	//clusters.erase(clusters.begin() + index);
	clusters = new_clusters;

}

std::vector<Cluster> ProrokClustering::ComputeClusters(const std::vector<Particle>& particles)
{
	int numPoints = particles.size();
	std::vector<Eigen::Vector3f> points(numPoints);
	std::vector<float> weights(numPoints);

	double w = 0.0;
	for(long unsigned int i = 0; i < numPoints; ++i)
	{
		w += particles[i].weight;
	}	
	 
	#pragma omp parallel for 
	for(long unsigned int i = 0; i < numPoints; ++i)
	{
		points[i] = particles[i].pose;
		weights[i] = particles[i].weight / w;
	}

	Cluster c0;
	c0.points = points;
	c0.weights = weights;
	std::vector<Cluster> clusters;
	clusters.push_back(c0);

	if (o_k == 1)
	{
		clusters[0].ComputeVariance();
	}

	for(long unsigned int i = 0; i < o_k - 1; ++i)
	{
		int dim = -1;
		int index = -1;
		findHighestVarianceCluster(clusters, index, dim);
		splitClusters(clusters, index, dim);
	}
	int dim = -1;
	int index = -1;
	findHighestVarianceCluster(clusters, index, dim);

	return clusters;
}

std::shared_ptr<DistributionData> ProrokClustering::Compress(const std::vector<Particle>& particles, Eigen::Vector3f trans)
{
	std::vector<Cluster> clusters = ComputeClusters(particles);

	std::vector<ClusterAbstraction> clusterAbstractions(o_k);

	float r = trans.head(2).norm();
	 #pragma omp parallel for 
	for(int i = 0; i < o_k; ++i)
	{
		clusterAbstractions[i] = ClusterAbstraction(clusters[i], trans);
		clusterAbstractions[i].covariance(0,0) += o_detectionNoise(0) * r;
		clusterAbstractions[i].covariance(1,1) += o_detectionNoise(1);
	}

	std::shared_ptr<DistributionData> distData = std::make_shared<DistributionData>(DistributionData(DISTRIBUTION::PROROK, clusterAbstractions));

	return distData;
}

void ProrokClustering::Fuse(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData)
{
	std::vector<ClusterAbstraction> clusterAbstractions = distData->Prorok();
	int k = clusterAbstractions.size();

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		Eigen::Vector3f p = particles[i].pose;
		double w = 0;

		for(int j = 0; j < k; ++j)
		{
			if (clusterAbstractions[j].weight < 0.01) continue;

			Eigen::Vector2f delta = ClusterAbstraction::computePolarDifference(clusterAbstractions[j].centroid, p);
			double dx = (delta(0) - clusterAbstractions[j].mu(0));
			double dy = (delta(1) - clusterAbstractions[j].mu(1));
			double sx = (clusterAbstractions[j].covariance(0, 0));
			double sy = (clusterAbstractions[j].covariance(1, 1));

			w += clusterAbstractions[j].weight * exp(-0.5 * ((dx * dx) / sx + (dy * dy) / sy));

		}
		particles[i].weight *= w ;
	}

	ReciprocalSampling(particles, distData);
}

void ProrokClustering::ReciprocalSampling(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData)
{
	std::vector<ClusterAbstraction> clusterAbstractions = distData->Prorok();
	int k = clusterAbstractions.size();

	std::vector<float> accW(k, 0.0);

	double w = 0;
	for(int j = 0; j < k; ++j)
	{
		w += clusterAbstractions[j].weight;
		accW[j] = w;
	}
	for(int j = 0; j < k; ++j)
	{
		clusterAbstractions[j].weight /= w;
		accW[j] /= w;
	}

	o_resampler->SetTH(10000000000); //always resample
	o_resampler->Resample(particles);


	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		if(drand48() < 0.06) 
		{
			int c = 0;
			float u = drand48();
			while(u > accW[c])
			{
				++c;
			}

			Eigen::Vector3f centroid = clusterAbstractions[c].centroid;
			float r = clusterAbstractions[c].mu(0);
			float theta = clusterAbstractions[c].mu(1);
			float dr = SampleGuassian(sqrt(clusterAbstractions[c].covariance(0, 0)));
			float dt = SampleGuassian(sqrt(clusterAbstractions[c].covariance(1, 1)));

			Eigen::Vector3f p;
			p(0) = centroid(0) + (r + dr) * cos(centroid(2) + theta + dt);
			p(1) = centroid(1) + (r + dr) * sin(centroid(2) + theta + dt);
			p(2) =  2 * M_PI * drand48() - M_PI;

			particles[i].pose = p;
			particles[i].weight = 1.0 / particles.size();
		}
	}


} 

