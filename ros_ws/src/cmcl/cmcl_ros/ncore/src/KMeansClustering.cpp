/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: KMeansClustering.cpp                                                  #
# ##############################################################################
**/

#include "KMeansClustering.h"
#include "Utils.h"
#include <iostream>

KMeansClustering::KMeansClustering(int k, int epochs, Eigen::Vector2f detectionNoise)
{
	o_k = k;
	o_epochs = epochs;
	o_detectionNoise = detectionNoise;
	o_resampler = std::make_shared<Resampling>(Resampling());
}


std::shared_ptr<DistributionData> KMeansClustering::Compress(const std::vector<Particle>& particles_, Eigen::Vector3f trans)
{
	std::vector<Particle> particles = computeDetectedParticles(particles_, trans);

	size_t n = particles.size();
	std::vector<Eigen::Vector3f> points(n); 
	std::vector<float> weights(n);
	std::vector<Cluster> clusters(o_k);  

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < n; ++i)
	{
		points[i] = particles[i].pose;
		weights[i] = particles[i].weight;
	}

	// initialize the clusters
	for (int c = 0; c < o_k; ++c) 
	{
		clusters[c].mean = points[rand() % n];
	}

	for (int e = 0; e < o_epochs; ++e)
	{
		AssignClusters(points, weights, clusters);
		RecomputeCentroids(clusters);
	}

	// incorporate detection error variance
	float r = trans.head(2).norm();
	float varX = abs(r * o_detectionNoise(0) * cos(o_detectionNoise(1)));
	float varY = abs(r * o_detectionNoise(0) * sin(o_detectionNoise(1)));

	#pragma omp parallel for 
	for (int c = 0; c < o_k; ++c) 
	{
		clusters[c].ComputeVariance();
		clusters[c].variance(0) += varX;
		clusters[c].variance(1) += varY;
		clusters[c].ComputeError();
		clusters[c].ComputeWeight();
	}




    std::shared_ptr<DistributionData>  distData = std::make_shared<DistributionData>(DistributionData(DISTRIBUTION::KMEANS, clusters));

    return distData;
}

void KMeansClustering::Fuse(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData)
{

	std::vector<Cluster> clusters = distData->KMeans();
	int k = clusters.size();
	size_t n = particles.size();

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < n; ++i)
	{
		Eigen::Vector3f p = particles[i].pose;
		double w = 0.0;

		for (int c = 0; c < k; ++c)
		{
			Eigen::Vector3f delta = p - clusters[c].mean;

			w += exp(- 0.5 * ((delta(0) * delta(0))/ clusters[c].variance(0) + ((delta(1) * delta(1))/ clusters[c].variance(1)))) * clusters[c].totalWeight;
		}

		particles[i].weight *= w ;
	}

	ReciprocalSampling(particles, distData);
}

void KMeansClustering::ReciprocalSampling(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData)
{
	std::vector<Cluster> clusters = distData->KMeans();
	int k = clusters.size();

	std::vector<float> accW(k, 0.0);
	double w = 0;
	for(int j = 0; j < k; ++j)
	{
		w += clusters[j].totalWeight;
		accW[j] = w;
	}
	for(int j = 0; j < k; ++j)
	{
		accW[j] /= w;
	}

	float alpha = 0.06;
	o_resampler->SetTH(10000000000); //always resample
	o_resampler->Resample(particles);	

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		if(drand48() < alpha) 
		{
			int c = 0;
			float u = drand48();
			while(u > accW[c])
			{
				++c;
			}

			float x = clusters[c].mean(0) + SampleGuassian(sqrt(clusters[c].variance(0)));
			float y = clusters[c].mean(1) + SampleGuassian(sqrt(clusters[c].variance(1)));

			/*while (!o_gmap->IsValid(Eigen::Vector3f(x,y, 0)))
			{
				c = 0;
				float u = drand48();
				while(u > accW[c])
				{
					++c;
				}
				x = clusters[c].mean(0) + SampleGuassian(sqrt(clusters[c].variance(0)));
				y = clusters[c].mean(1) + SampleGuassian(sqrt(clusters[c].variance(1)));	
			}*/
			
			particles[i].pose(0) = x;
			particles[i].pose(1) = y;
			particles[i].pose(2) = 2 * M_PI * drand48() - M_PI;
			particles[i].weight =  1 / particles.size(); 
		}
	}
}





void KMeansClustering::AssignClusters(std::vector<Eigen::Vector3f>& points, std::vector<float>& weights, std::vector<Cluster>& clusters)
{
	int pointNum = points.size();
	int k = clusters.size();

	std::vector<float> minDistance(pointNum, __FLT_MAX__);
	std::vector<uint> clusterID(pointNum, 0);

	#pragma omp parallel for
	for (int c = 0; c < k; ++c)
	{
		clusters[c].points.clear();
	}

	#pragma omp parallel for
	for (int i = 0; i < pointNum; ++i)
	{
		//uint cID = 0;
		for (int c = 0; c < k; ++c)
		{
			Eigen::Vector2f diff = points[i].head(2) - clusters[c].mean.head(2);
			float dist = diff.squaredNorm();

			if (dist < minDistance[i]) 
			{
	            minDistance[i] = dist;
	            clusterID[i] = c;
        	}
		}
	}

	for (int i = 0; i < pointNum; ++i)
	{
		clusters[clusterID[i]].points.push_back(points[i]);
		clusters[clusterID[i]].weights.push_back(weights[i]);
	}
}

void KMeansClustering::RecomputeCentroids(std::vector<Cluster>& clusters)
{
	int k = clusters.size();

	#pragma omp parallel for
	for (int c = 0; c < k; ++c)
	{
		clusters[c].ComputeMean();
	}	
}

