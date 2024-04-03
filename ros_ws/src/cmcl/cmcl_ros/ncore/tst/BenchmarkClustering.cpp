/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: BenchmarkClustering.cpp   		                           		       #
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


void testGoodpoints(std::vector<Particle> particlesA, std::vector<Particle> particlesB)
{
	GoodPointsCompression gp("/ros_ws/src/cmcl/cmcl_ros/ncore/src/", Eigen::Vector2f(0.15, 0.15));

    int iter = 100;
	auto millis = 0;

	std::shared_ptr<DistributionData> distData;

	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
		distData = gp.Compress(particlesA, Eigen::Vector3f(0, 0, 0));
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "Goodpoints Compress: " << (float)millis / (iter * 1000) << " ms" << std::endl;
	//std::cout << distData->Particles().size() << std::endl;

	millis = 0;

	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
		gp.Fuse(particlesB, distData);
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "Goodpoints Fuse: " << (float)millis / (iter * 1000) << " ms" << std::endl;
}



void testKmeans(std::vector<Particle> particlesA, std::vector<Particle> particlesB)
{
	KMeansClustering km(64, 5, Eigen::Vector2f(0, 0));
    int iter = 100;
	auto millis = 0;
	std::shared_ptr<DistributionData> distData;

	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
	    distData = km.Compress(particlesA, Eigen::Vector3f(0, 0, 0));
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "Kmeans Compress: " << (float)millis / (iter * 1000) << " ms" << std::endl;

	millis = 0;
	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
		km.Fuse(particlesB, distData);
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "Kmeans Fuse: " << (float)millis / (iter * 1000) << " ms" << std::endl;
}

void testProrok(std::vector<Particle> particlesA, std::vector<Particle> particlesB)
{
	ProrokClustering pc(64, Eigen::Vector2f(0.15, 0.15));
    int iter = 100;
	auto millis = 0;
	std::shared_ptr<DistributionData> distData;
	Eigen::Vector3f trans(3.0, -2.0, 0.98);


	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
		distData = pc.Compress(particlesA, trans);
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "Prorok Compress: " << (float)millis / (iter * 1000) << " ms" << std::endl;

	millis = 0;
	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
		pc.Fuse(particlesB, distData);
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "Prorok Fuse: " << (float)millis / (iter * 1000) << " ms" << std::endl;

}

void testDET(std::vector<Particle> particlesA, std::vector<Particle> particlesB)
{
	DensityEstimationTree det(150, 200);
    int iter = 100;
	auto millis = 0;
	std::shared_ptr<DistributionData> distData;

	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
		distData = det.Compress(particlesA, Eigen::Vector3f(0, 0, 0));
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "DET Compress: " << (float)millis / (iter * 1000) << " ms" << std::endl;

	millis = 0;
	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
		det.Fuse(particlesB, distData);
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "DET Fuse: " << (float)millis / (iter * 1000) << " ms" << std::endl;

}

void testNaive(std::vector<Particle> particlesA, std::vector<Particle> particlesB)
{
	StandardThinning st(10000, Eigen::Vector2f(0.15, 0.15));
    int iter = 100;
	auto millis = 0;
	std::shared_ptr<DistributionData> distData;

	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
		distData = st.Compress(particlesA, Eigen::Vector3f(0, 0, 0));
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "Naive Compress: " << (float)millis / (iter * 1000) << " ms" << std::endl;

	millis = 0;
	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
		st.Fuse(particlesB, distData);
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "Naive Fuse: " << (float)millis / (iter * 1000) << " ms" << std::endl;

}

void testThin(std::vector<Particle> particlesA, std::vector<Particle> particlesB)
{
	StandardThinning st(64, Eigen::Vector2f(0.15, 0.15));
    int iter = 100;
	auto millis = 0;
	std::shared_ptr<DistributionData> distData;

	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
		distData = st.Compress(particlesA, Eigen::Vector3f(0, 0, 0));
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "Thin Compress: " << (float)millis / (iter * 1000) << " ms" << std::endl;

	millis = 0;
	for(int i = 0; i < iter; ++i)
	{
		auto start = std::chrono::system_clock::now();
		st.Fuse(particlesB, distData);
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> dur = end - start;
		millis += std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
	}
	std::cout << "Thin Fuse: " << (float)millis / (iter * 1000) << " ms" << std::endl;
}


int main()
{
	int n = 10000;
	int k = 4;
	int s = 1;
	int d = 0;
	std::string csv_pathA = "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/" + std::to_string(n) + "/" + std::to_string(s) + "/particlesA_" + std::to_string(d) + ".csv";
	std::vector<Particle> particlesA = load_csv(csv_pathA);

	std::string csv_pathB = "/ros_ws/src/cmcl/cmcl_ros/ncore/tst/data/" + std::to_string(n) + "/" + std::to_string(s) + "/particlesB_" + std::to_string(d) + ".csv";
	std::vector<Particle> particlesB = load_csv(csv_pathB);

	std::cout << particlesA.size() <<  ", " << particlesB.size() << std::endl;

	testNaive(particlesA, particlesB);
	testGoodpoints(particlesA, particlesB);
	testKmeans(particlesA, particlesB);
	testDET(particlesA, particlesB);
	testThin(particlesA, particlesB);
	testProrok(particlesA, particlesB);

	

	return 0;
}