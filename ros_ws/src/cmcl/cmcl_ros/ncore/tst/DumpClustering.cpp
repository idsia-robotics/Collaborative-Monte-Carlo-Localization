/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: DumpClustering.cpp      		                           		       #
# ##############################################################################
**/


#include <iostream>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <chrono>

#include "DistributionExchange.h"
#include "TestUtils.h"

void dumpProrok(int k, std::vector<Particle>& particles, const std::string& dumpPath)
{
	Eigen::Vector3f trans(0.0, 0.0, 0.0);
	ProrokClustering pc(k, Eigen::Vector2f(0.15, 0.15));

	std::shared_ptr<DistributionData> distData = pc.Compress(particles, trans);

	std::vector<Eigen::Vector3f> points(k);

	for(int c = 0; c < k; ++c)
	{
		points[c] = distData->Prorok()[c].centroid;
		//std::cout << distData->Prorok()[c].centroid << std::endl;
	}

	dump_csv(points, dumpPath);
}

void dumpKMeans(int k, std::vector<Particle>& particles, const std::string& dumpPath)
{
	KMeansClustering km(k, 5, Eigen::Vector2f(0, 0));

	std::shared_ptr<DistributionData> distData = km.Compress(particles, Eigen::Vector3f(0, 0, 0));

	std::vector<Eigen::Vector3f> points(k);

	for(int c = 0; c < k; ++c)
	{
		points[c] = distData->KMeans()[c].mean;
		//std::cout << distData->Prorok()[c].centroid << std::endl;
	}

	dump_csv(points, dumpPath);
}

void dumpGoodPoints(std::vector<Particle>& particles, const std::string& dumpPath)
{
	Eigen::Vector3f trans(0.0, 0.0, 0.0);
	GoodPointsCompression gp("/ros_ws/src/cmcl/cmcl_ros/ncore/src/", Eigen::Vector2f(0.15, 0.15));

	std::shared_ptr<DistributionData> distData = gp.Compress(particles, trans);

	dump_csv(distData->Particles(), dumpPath);
}

void dumpThin(int n, std::vector<Particle>& particles, const std::string& dumpPath)
{
	int m = particles.size();
	std::vector<Particle> subset(n);
	for(int j = 0; j < n; ++j)
	{
		subset[j] = particles[(int)(drand48() * m)];
	}

	dump_csv(subset, dumpPath);
}


int main()
{
	int n = 20;
	int k = 4;
	int s = 1;
	int d = 0;

	srand48(13);

	std::vector<Particle> particles = createSymmetricXDist(n, k);
	std::string csv_path = "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/fail/particlesA_0.csv";
	dump_csv(particles, csv_path);

	dumpProrok(8, particles, "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/fail/prorok_dump.csv");
	dumpKMeans(8, particles, "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/fail/kmeans_dump.csv");
	dumpGoodPoints(particles, "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/fail/goodpoints_dump.csv");
	dumpThin(8, particles, "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/fail/thin_dump.csv");

	particles = createSymmetricXDistNoCenter(n, k);
	csv_path = "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/success/particlesA_0.csv";
	dump_csv(particles, csv_path);

	dumpProrok(8, particles, "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/success/prorok_dump.csv");
	dumpKMeans(8, particles, "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/success/kmeans_dump.csv");
	dumpGoodPoints(particles, "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/success/goodpoints_dump.csv");
	dumpThin(8, particles, "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/success/thin_dump.csv");

	return 0;
}

