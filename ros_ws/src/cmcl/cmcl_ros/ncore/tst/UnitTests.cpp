/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: UnitTests.cpp             		                           		       #
# ##############################################################################
**/



#include "gtest/gtest.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <string>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <string>



#include "Utils.h"
#include "FSR.h"
#include "GMap.h"
#include "Lidar2D.h"
#include "SetStatistics.h"
#include "LidarData.h"
#include "BeamEnd.h"
#include "MCL.h"

#include "DistributionExchange.h"
#include "ProrokClustering.h"
#include "KMeansClustering.h"


#include <pybind11/embed.h>
#include <eigen3/Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

// std::string dataPath = PROJECT_TEST_DATA_DIR + std::string("/8/");
// std::string testPath = PROJECT_TEST_DATA_DIR + std::string("/test/floor/");
// std::string simplePath = PROJECT_TEST_DATA_DIR + std::string("/test/8Map/");
std::string configDir = PROJECT_TEST_DATA_DIR + std::string("/config/");


/* FSR */
/*
TEST(TestFSR, test1) {
    
    Eigen::Vector3f p1 = Eigen::Vector3f(1.2, -2.5, 0.67);
	Eigen::Vector3f p2 = Eigen::Vector3f(-1.5, 4.3, -1.03);

	FSR fsr = FSR();
	Eigen::Vector3f u = fsr.Backward(p1, p2);
	Eigen::Vector3f p2_comp = fsr.Forward(p1, u);
	ASSERT_EQ(p2_comp, p2);
}

TEST(TestFSR, test2) {

	FSR fsr = FSR();
	//test against know python result
	Eigen::Vector3f u2_gt = Eigen::Vector3f(-13.755235632332337, -3.971585665576746, 0.62);
	Eigen::Vector3f u2 = fsr.Backward(Eigen::Vector3f(13.6, -6.1, -0.33), Eigen::Vector3f(-0.7, -5.4, 0.29));
	ASSERT_NEAR(u2_gt(0), u2(0), 0.001);
	ASSERT_NEAR(u2_gt(1), u2(1), 0.001);
	ASSERT_NEAR(u2_gt(2), u2(2), 0.001);
}

TEST(TestFSR, test3) {

	FSR mfsr = FSR();
	//float choose  = drand48();
	//test against know python result
	Eigen::Vector3f p_gt = Eigen::Vector3f(13.61489781, -6.21080639, -0.31);
	std::vector<float> commandWeights{0.5f, 0.5f};

	std::vector<Eigen::Vector3f> command{Eigen::Vector3f(0.05, -0.1, 0.02), Eigen::Vector3f(0.05, -0.1, 0.02)};
	Eigen::Vector3f noisy_p = mfsr.SampleMotion(Eigen::Vector3f(13.6, -6.1, -0.33), command, commandWeights, Eigen::Vector3f(0.0, 0.0, 0.0));
	ASSERT_NEAR(noisy_p(0), p_gt(0), 0.001);
	ASSERT_NEAR(noisy_p(1), p_gt(1), 0.001);
	ASSERT_NEAR(noisy_p(2), p_gt(2), 0.001);
	
}

TEST(TestGMap, test1) {
    
    Eigen::Vector3f origin = Eigen::Vector3f(-12.200000, -17.000000, 0.000000);
	float resolution = 0.05;
	cv::Mat gridMap = cv::imread(dataPath + "YouBotMap.pgm");
	GMap gmap = GMap(gridMap, origin, resolution);

	Eigen::Vector2f o = gmap.World2Map(Eigen::Vector2f(0, 0));
	ASSERT_EQ(o, Eigen::Vector2f(244, 332));
}

TEST(TestGMap, test2) {

	Eigen::Vector3f origin = Eigen::Vector3f(-12.200000, -17.000000, 0.000000);
	float resolution = 0.05;
	cv::Mat gridMap = cv::imread(dataPath + "YouBotMap.pgm");
	GMap gmap = GMap(gridMap, origin, resolution);

	Eigen::Vector2f p = gmap.Map2World(Eigen::Vector2f(244, 332));
	ASSERT_EQ(p, Eigen::Vector2f(0, 0));
}


TEST(TestGMap, test3) {

	std::string mapFolder = dataPath;
	GMap gmap = GMap(mapFolder);

	Eigen::Vector2f p = gmap.Map2World(Eigen::Vector2f(244, 332));
	ASSERT_EQ(p, Eigen::Vector2f(0, 0));
}

TEST(TestGMap, test4) {

   std::string mapFolder = dataPath;
    GMap gmap = GMap(mapFolder);

    Eigen::Vector2f p = gmap.Map2World(Eigen::Vector2f(244, 332));
    Eigen::Vector2f pm = gmap.World2Map(p);

    ASSERT_EQ(pm(0), 244);
    ASSERT_EQ(pm(1), 332);
}

TEST(TestGMap, test5)
{

    std::string mapFolder = dataPath;
    GMap gmap = GMap(mapFolder);

    Eigen::Vector2f p = gmap.Map2World(Eigen::Vector2f(244, 332));
    bool valid = gmap.IsValid(Eigen::Vector3f(p(0), p(1), 0));

    ASSERT_EQ(true, valid);
}

TEST(TestGMap, test6)
{

    std::string mapFolder = dataPath;
    GMap gmap = GMap(mapFolder);

    Eigen::Vector2f p = gmap.Map2World(Eigen::Vector2f(0, 0));
    bool valid = gmap.IsValid(Eigen::Vector3f(p(0), p(1), 0));

    ASSERT_EQ(false, valid);
}


TEST(TestUtils, test1) {
    
    float angle = 2 * M_PI;
	//should be 0 or near
    float angle_wrap = Wrap2Pi(angle);
	ASSERT_NEAR(0.0, angle_wrap, 0.000001);
}

TEST(TestUtils, test2) {
    
	// identity martrix
	Eigen::Matrix3f identity = Eigen::Matrix3f::Identity(3, 3);
	Eigen::Matrix3f v2t = Vec2Trans(Eigen::Vector3f(0, 0, 0));
	ASSERT_EQ(identity, v2t);

}

TEST(TestUtils, test3) {
    
	// from python verified code, should be -1.5707963267948966
	float yaw = GetYaw(0.7, -0.7);
	ASSERT_NEAR(yaw, -1.5707963267948966, 0.00001);

}
*/

TEST(TestUtils, test4) {
    
	std::vector<float> ranges{1.0, 2.0, 3.0};
	std::vector<float> angles{1.0, 0.0, -0.5};
	std::vector<Eigen::Vector3f> hp = Ranges2Points(ranges, angles);

	// from python verified code
	/*
		hp = [[ 0.54030231  2.          2.63274769]
		 	  [ 0.84147098  0.         -1.43827662]
		 	  [ 1.          1.          1.        ]]
	*/

	std::vector<Eigen::Vector3f> hp_gt{Eigen::Vector3f(0.54030231, 0.84147098, 1.0),
										Eigen::Vector3f(2.0, 0.0, 1.0), 
										Eigen::Vector3f(2.63274769, -1.43827662, 1.0)};

	ASSERT_NEAR(hp[0](0), hp_gt[0](0), 0.001);
	ASSERT_NEAR(hp[0](1), hp_gt[0](1), 0.001);
	ASSERT_NEAR(hp[0](2), hp_gt[0](2), 0.001);
	ASSERT_NEAR(hp[1](0), hp_gt[1](0), 0.001);
	ASSERT_NEAR(hp[1](1), hp_gt[1](1), 0.001);
	ASSERT_NEAR(hp[1](2), hp_gt[1](2), 0.001);
	ASSERT_NEAR(hp[2](0), hp_gt[2](0), 0.001);
	ASSERT_NEAR(hp[2](1), hp_gt[2](1), 0.001);
	ASSERT_NEAR(hp[2](2), hp_gt[2](2), 0.001);

}


TEST(TestCluster, test1)
{
	std::vector<float> weights(5, 1.0);
	std::vector<Eigen::Vector3f> points = {Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0.005, 1.0, 0), Eigen::Vector3f(0.01, 0.7, 0), Eigen::Vector3f(-0.01, 2.0, 0), Eigen::Vector3f(0, -1.5, 0)};
	Cluster c;
	c.weights = weights;
	c.points = points;
	c.ComputeVariance();

	ASSERT_NEAR(c.mean(0), 0.001 , 0.01);
	ASSERT_NEAR(c.mean(1), 0.44 , 0.01);

}

TEST(TestProrokClustering, test1)
{
	std::vector<float> weights(5, 1.0);
	std::vector<Eigen::Vector3f> points = {Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0.005, 1.0, 0), Eigen::Vector3f(0.01, 0.7, 0), Eigen::Vector3f(-0.01, 2.0, 0), Eigen::Vector3f(0, -1.5, 0)};
	Cluster c1;
	c1.weights = weights;
	c1.points = points;
	c1.ComputeVariance();



	Cluster c2;
	c2.weights = weights;
	c2.points = std::vector<Eigen::Vector3f>(6, Eigen::Vector3f(-1, 1, 0));
	c2.ComputeVariance();

	std::vector<Cluster> clusters;
	clusters.push_back(c1);
	clusters.push_back(c2);

	int index = -1;
	int dim = -1;

	ProrokClustering pc(2);
	pc.findHighestVarianceCluster(clusters, index, dim);

	ASSERT_EQ(index, 0);
	ASSERT_EQ(dim, 1);
}



TEST(TestProrokClustering, test2)
{

	int n = 1000;

	std::vector<float> weights(n, 1.0);
	Cluster c1, c2, c3;	
	c1.weights = weights;
	c2.weights = weights;
	c3.weights = weights;

	for (int i = 0; i < n; ++i)
	{
		Eigen::Vector3f p(-0.5 + drand48(), -0.5 + drand48(), 0);
		c1.points.push_back(p);
	}

	for (int i = 0; i < n; ++i)
	{
		Eigen::Vector3f p(0 + 0.001 * drand48(), 0 + 2.0 * drand48(), 0);
		c2.points.push_back(p);
	}

	for (int i = 0; i < n; ++i)
	{
		Eigen::Vector3f p(1 + 3.0 * drand48(), -1 + 5.0 * drand48(), 0);
		c3.points.push_back(p);
	}
	
	std::vector<Cluster> clusters;
	clusters.push_back(c1);
	clusters.push_back(c2);
	clusters.push_back(c3);

	int index = -1;
	int dim = -1;
	int k = 3;

	ProrokClustering pc(k);
	pc.findHighestVarianceCluster(clusters, index, dim);

	//for (int i = 0; i < k; ++i)
	//{
	//	std::cout << clusters[i].variance(0) << ", " <<  clusters[i].variance(1) << std::endl;
	//}

	ASSERT_EQ(index, 2);
	ASSERT_EQ(dim, 1);

	int size  = clusters.size();
	//std::cout << clusters[2].points.size() << std::endl;


	pc.splitClusters(clusters, index, dim);

	int size2  = clusters.size();

	ASSERT_EQ(size + 1, size2);

	//std::cout << clusters[2].points.size() << std::endl;
	//std::cout << clusters[3].points.size() << std::endl;

}



TEST(TestProrokClustering, test3)
{

	int numParticles = 5000;
	int k = 5;
	size_t perCluster = numParticles / k;
	Eigen::Vector3f trans(0,0,0);
	std::vector<float> weights(numParticles, 1.0);
	std::vector<Particle> particles(numParticles);


	std::vector<Eigen::Vector3f> centroids = {Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(4.0, 5.0, 1.0), Eigen::Vector3f(6.0, -6.0, -2.0),
												Eigen::Vector3f(-6.0, 5.0, -1.2), Eigen::Vector3f(-5.0, -6.0, 0.4)};

	std::vector<Eigen::Vector3f> points(numParticles);

	for(int j = 0; j < k; ++j)
	{
		for (long unsigned int i = 0; i < perCluster; ++i)
		{
			//points[j * perCluster + i] = 0.1 * Eigen::Vector3f(drand48(), drand48(), drand48()) + centroids[j];
			particles[j * perCluster + i].pose = 0.1 * Eigen::Vector3f(drand48(), drand48(), drand48()) + centroids[j];
			particles[j * perCluster + i].weight = 1.0;
		}
	}

	ProrokClustering pc(k);
	std::vector<Cluster> clusters = pc.ComputeClusters(particles);


	//std::vector<Cluster> clusters = ProrokClustering(points, weights, k);

	// for (int i = 0; i < k; ++i)
	// {
	// 	std::cout << "mean: " << clusters[i].mean(0) << ", " <<  clusters[i].mean(1) << std::endl;
	// 	std::cout << "variance: " << clusters[i].variance(0) << ", " <<  clusters[i].variance(1) << std::endl;
	// }

	ASSERT_EQ(clusters.size(), k);
}

TEST(TestGoodPoints, test1)
{
	int n = 1000;

	std::vector<Particle> particles(n);
	for (int i = 0; i < n; ++i)
	{
		particles[i].pose = Eigen::Vector3f(0, 0, 0);
		particles[i].weight = 1.0;
	}

	GoodPointsCompression gp("/ros_ws/src/cmcl/cmcl_ros/ncore/src/");

	std::shared_ptr<DistributionData> distData = gp.Compress(particles, Eigen::Vector3f(1.0, 0.2, 0.0));	

}


TEST(TestKMeansClustering, test1)
{
	int k = 1;
	int n = 1000;
	int epochs = 1;
	std::vector<Particle> particles(n);

	for (int i = 0; i < n; ++i)
	{
		particles[i].pose = Eigen::Vector3f(-0.5 + drand48(), -0.5 + drand48(), 0.01 * drand48());
		particles[i].weight = 1.0;
	}

	KMeansClustering kc(k, epochs, Eigen::Vector2f(0, 0));

	std::shared_ptr<DistributionData> distData = kc.Compress(particles, Eigen::Vector3f(0, 0, 0));
	std::vector<Cluster> clusters = distData->KMeans();
	
	ASSERT_NEAR(clusters[0].mean(0), 0.0 , 0.015);
	ASSERT_NEAR(clusters[0].mean(1), 0.0 , 0.015);
}


TEST(TestKMeansClustering, test2)
{
	int k = 2;
	int n = 1000;
	int epochs = 10;
	std::vector<Particle> particles(k * n);

	for (int i = 0; i < n; ++i)
	{
		Eigen::Vector3f rand = Eigen::Vector3f::Ones() - 2.0 * Eigen::Vector3f(drand48(), drand48(), drand48());
		particles[i].pose = Eigen::Vector3f(-0.5, -0.5, 0.0) + 0.1 * rand;
		particles[i].weight = 1.0;
	}

	for (int i = 0; i < n; ++i)
	{
		Eigen::Vector3f rand = Eigen::Vector3f::Ones() - 2.0 * Eigen::Vector3f(drand48(), drand48(), drand48());
		particles[n + i].pose = Eigen::Vector3f(2.0 , 2.0 , 0.0) + 0.1 * rand;
		particles[n + i].weight = 1.0;
	}

	std::vector<Eigen::Vector3f> points(k * n); 
	std::vector<float> weights(k * n); 


	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		points[i] = particles[i].pose;
		weights[i] = particles[i].weight;
	}


	KMeansClustering kc(k, epochs, Eigen::Vector2f(0, 0));
	std::vector<Cluster> clusters(k); 


	// initialize the clusters
	for (int i = 0; i < k; ++i) 
	{
		clusters[i].mean = points[10 + i * n];
	}

	kc.AssignClusters(points, weights, clusters);
	kc.RecomputeCentroids(clusters);

	for (int i = 0; i < k; ++i) 
	{
		std::cout << "cluster " << i << ": " << clusters[i].mean(0) << ", " << clusters[i].mean(1) << ", " << clusters[i].mean(2) << std::endl;
	}

	if (clusters[0].mean.head(2).norm() > clusters[1].mean.head(2).norm())
	{
		ASSERT_NEAR(clusters[0].mean(0), 2.0 , 0.05);
		ASSERT_NEAR(clusters[0].mean(1), 2.0 , 0.05);
		ASSERT_NEAR(clusters[1].mean(0), -0.5 , 0.05);
		ASSERT_NEAR(clusters[1].mean(1), -0.5 , 0.05);
		
	}
	else
	{
		ASSERT_NEAR(clusters[1].mean(0), 2.0 , 0.05);
		ASSERT_NEAR(clusters[1].mean(1), 2.0 , 0.05);
		ASSERT_NEAR(clusters[0].mean(0), -0.5 , 0.05);
		ASSERT_NEAR(clusters[0].mean(1), -0.5 , 0.05);
	}

}


TEST(TestDET, test1)
{
	
	size_t numParticles = 1000;
	std::vector<Particle> particles(numParticles);

	for (long unsigned int i = 0; i < numParticles; ++i)
	{
		Eigen::Vector3f rand =  (Eigen::Vector3f::Ones() - 2.0 * Eigen::Vector3f(drand48(), drand48(), drand48()));
		particles[i].pose = Eigen::Vector3f(1.0, -1.5, 0.3) + 0.1 * rand;
		particles[i].weight = 1.0;
		//std::cout << particles[i].pose(0) << ", " << particles[i].pose(1) << std::endl;
	}

	DensityEstimationTree det(5, 10);
	std::shared_ptr<mlpack::DTree<arma::mat>>  tree = det.TrainDET(particles);

	Particle p;
	p.pose = Eigen::Vector3f(0.99, -1.49, 0.31);
	double w = det.QueryDET(tree, p);
	Particle p2;
	p2.pose = Eigen::Vector3f(0.85, -1.25, 0.25);
	double w2 = det.QueryDET(tree, p2);

    ASSERT_GE(w, w2);
}

TEST(TestDistributionExchange, test1)
{
	
	size_t numParticles = 1000;
	std::vector<Particle> particles(numParticles);

	for (long unsigned int i = 0; i < numParticles; ++i)
	{
		Eigen::Vector3f rand = 0.5 * (Eigen::Vector3f::Ones() - 2.0 * Eigen::Vector3f(drand48(), drand48(), drand48()));
		particles[i].pose = Eigen::Vector3f(1.0, -1.5, 0.3) + rand;
		particles[i].weight = 1.0;
		//std::cout << particles[i].pose(0) << ", " << particles[i].pose(1) << std::endl;
	}

	DensityEstimationTree det(50, 100);
	std::shared_ptr<DistributionData> distData = det.Compress(particles, Eigen::Vector3f(0, 0, 0));

	std::vector<Particle> particles2(2);
	particles2[0].pose = Eigen::Vector3f(1.00, -1.5, 0.30);
	particles2[0].weight = 1.0;

	particles2[1].pose = Eigen::Vector3f(1.50, -1.7, 0.40);
	particles2[1].weight = 1.0;

	det.Fuse(particles2, distData);

    ASSERT_GE(particles2[0].weight, particles2[1].weight);
}

TEST(TestDistributionExchange, test2)
{
	
	DistributionExchange o_distExcange;

	size_t numParticles = 1000;
	std::vector<Particle> particles(numParticles);

	for (long unsigned int i = 0; i < numParticles; ++i)
	{
		particles[i].pose = Eigen::Vector3f(drand48(), drand48(), drand48());
	}

	std::shared_ptr<DistributionData> distData = o_distExcange.CreateDistribution(particles, DISTRIBUTION::PARTICLES);

	std::cout << distData->Particles().size() << std::endl;
}

TEST(TestDistributionExchange, test3)
{
	int k = 5;
	std::shared_ptr<DistributionExchange> o_distExcange = std::make_shared<DistributionExchange>(DistributionExchange(k));

	size_t numParticles = 5000;
	size_t perCluster = numParticles / k;
	Eigen::Vector3f trans(0,0,0);

	std::vector<Eigen::Vector3f> centroids = {Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(4.0, 5.0, 1.0), Eigen::Vector3f(6.0, -6.0, -2.0),
												Eigen::Vector3f(-6.0, 5.0, -1.2), Eigen::Vector3f(-5.0, -6.0, 0.4)};

	std::vector<Particle> particles(numParticles);


	for(int j = 0; j < k; ++j)
	{
		for (long unsigned int i = 0; i < perCluster; ++i)
		{
			particles[j * perCluster + i].pose = 0.1 * Eigen::Vector3f(drand48(), drand48(), drand48()) + centroids[j];
			particles[j * perCluster + i].weight = 1.0;
		}
	}

	std::shared_ptr<DistributionData> distData = o_distExcange->CreateDistribution(particles, DISTRIBUTION::PROROK, trans);

	std::vector<ClusterAbstraction> abstractions = distData->Prorok();
	ASSERT_EQ(abstractions.size(), k);

	/*for(int i = 0; i < k; ++i)
	{
		std::cout << abstractions[i].centroid(0) << ", " << abstractions[i].centroid(1) << ", " << abstractions[i].centroid(2) << std::endl;
	}*/

	

}


TEST(TestDistributionExchange, test4)
{
	int k = 1;
	std::shared_ptr<DistributionExchange> o_distExcange = std::make_shared<DistributionExchange>(DistributionExchange(k));

	size_t numParticles = 200;
	size_t perCluster = numParticles / k;
	Eigen::Vector3f trans(1,1,0);

	std::vector<Eigen::Vector3f> centroids = {Eigen::Vector3f(0, 0, 0)};
	std::vector<Particle> particles(numParticles);
	for(int j = 0; j < k; ++j)
	{
		for (long unsigned int i = 0; i < perCluster; ++i)
		{
			particles[j * perCluster + i].pose = 0.1 * Eigen::Vector3f(drand48(), drand48(), drand48()) + centroids[j];
			particles[j * perCluster + i].weight = 1.0;
		}
	}

	std::shared_ptr<DistributionData> distData = o_distExcange->CreateDistribution(particles, DISTRIBUTION::PROROK, trans);

	std::vector<ClusterAbstraction> abstractions = distData->Prorok();
	ASSERT_EQ(abstractions.size(), k);

	// for (int i = 0; i < k; ++i)
	// {
	// 	std::cout << "centroid:" << abstractions[i].centroid(0) << ", " <<  abstractions[i].centroid(1) << ", " <<  abstractions[i].centroid(2) << std::endl;
	// 	std::cout << "mu:" << abstractions[i].mu(0) << ", " <<  abstractions[i].mu(1) << std::endl;
	// 	std::cout << "covariance:" << abstractions[i].covariance(0,0) << ", " <<  abstractions[i].covariance(1,1) << std::endl;
	// 	std::cout << "weight:" << abstractions[i].weight << std::endl;
	// }


	int numParticles2 = 2;
	std::vector<Particle> particles2(numParticles2);
	particles2[0].pose = trans;
	particles2[0].weight = 1.0;
	particles2[1].pose = Eigen::Vector3f(-10, 4, 0);
	particles2[1].weight = 1.0;

	o_distExcange->FuseDistribution(particles2, distData);

	// for (int i = 0; i < numParticles2; ++i)
	// {
	// 	std::cout << particles2[i].pose(0) << ", " << particles2[i].pose(1) << std::endl;
	// }

	ASSERT_NEAR(particles2[0].weight, 0.5, 0.05);
	ASSERT_NEAR(particles2[0].pose(0), trans(0), 0.05);
	ASSERT_NEAR(particles2[0].pose(1), trans(1), 0.05);
	ASSERT_NEAR(particles2[1].weight, 0.5, 0.05);

}

TEST(TestDistributionExchange, test5)
{
	int k = 3;
	std::shared_ptr<DistributionExchange> o_distExcange = std::make_shared<DistributionExchange>(DistributionExchange(k));

	size_t numParticles = 1000;
	Eigen::Vector3f trans(1,1,0);
	std::vector<Particle> particles(numParticles);
	
	for (int i = 0; i < 700; ++i)
	{
		particles[i].pose = 0.1 * Eigen::Vector3f(drand48(), drand48(), drand48()) + Eigen::Vector3f(0, 0, 0);
		particles[i].weight = 1.0;
	}

	for (int i = 0; i < 100; ++i)
	{
		particles[700 + i].pose = 0.1 * Eigen::Vector3f(drand48(), drand48(), drand48()) + Eigen::Vector3f(10, -5, 0);
		particles[700 + i].weight = 1.0;
	}

	for (int i = 0; i < 200; ++i)
	{
		particles[800 + i].pose = 0.1 * Eigen::Vector3f(drand48(), drand48(), drand48()) + Eigen::Vector3f(-3, 7, 0);
		particles[800 + i].weight = 1.0;
	}
	

	std::shared_ptr<DistributionData> distData = o_distExcange->CreateDistribution(particles, DISTRIBUTION::PROROK, trans);
	std::vector<ClusterAbstraction> abstractions = distData->Prorok();
	ASSERT_EQ(abstractions.size(), k);

	/*for (int i = 0; i < k; ++i)
	{
		std::cout << "centroid:" << abstractions[i].centroid(0) << ", " <<  abstractions[i].centroid(1) << ", " <<  abstractions[i].centroid(2) << std::endl;
		std::cout << "mu:" << abstractions[i].mu(0) << ", " <<  abstractions[i].mu(1) << std::endl;
		std::cout << "covariance:" << abstractions[i].covariance(0,0) << ", " <<  abstractions[i].covariance(1,1) << std::endl;
		std::cout << "weight:" << abstractions[i].weight << std::endl;
	}*/


	int numParticles2 = 2;
	std::vector<Particle> particles2(numParticles2);
	particles2[0].pose = trans;
	particles2[0].weight = 1.0f;
	particles2[1].pose = Eigen::Vector3f(-10, 4, 0);
	particles2[1].weight = 1.0f;

	o_distExcange->FuseDistribution(particles2, distData);
	for (int i = 0; i < numParticles2; ++i)
	{
		std::cout << particles2[i].pose(0) << ", " << particles2[i].pose(1) << ", " << particles2[i].weight << std::endl;
	}

	float norm = (particles2[0].pose.head(2) - particles2[1].pose.head(2)).norm(); 

	ASSERT_NEAR(particles2[0].weight, 0.5, 0.05);
	ASSERT_NEAR(particles2[1].weight, 0.5, 0.05);
	ASSERT_NEAR(norm, 0.0, 0.05);
}


TEST(TestDistributionExchange, test6)
{
	int k = 2;
	int n = 1000;
	int epochs = 5;
	std::vector<Particle> particles(k * n);

	for (int i = 0; i < n; ++i)
	{
		Eigen::Vector3f rand = Eigen::Vector3f::Ones() - 2.0 * Eigen::Vector3f(drand48(), drand48(), drand48());
		particles[i].pose = Eigen::Vector3f(-0.5, -0.5, 0.0) + rand;
		particles[i].weight = 1.0;
	}

	for (int i = 0; i < n; ++i)
	{
		Eigen::Vector3f rand = Eigen::Vector3f::Ones() - 2.0 * Eigen::Vector3f(drand48(), drand48(), drand48());
		particles[n + i].pose = Eigen::Vector3f(2.0 , 2.0 , 0.0) + rand;
		particles[n + i].weight = 1.0;
	}

	KMeansClustering kc(k, epochs, Eigen::Vector2f(0, 0));
	std::shared_ptr<DistributionData> distData = kc.Compress(particles, Eigen::Vector3f(0, 0, 0));

	int numParticles2 = 2;
	std::vector<Particle> particles2(numParticles2);
	particles2[0].pose = Eigen::Vector3f(1.97, 2.04, 0);
	particles2[0].weight = 1.0;
	particles2[1].pose = Eigen::Vector3f(-10, 4, 0);
	particles2[1].weight = 1.0;

	kc.Fuse(particles2, distData);

	ASSERT_GE(particles2[0].weight, particles2[1].weight);	
}

TEST(TestDistributionExchange, test7)
{
	std::string configPath = configDir + "cmcl.config";
	std::shared_ptr<DistributionExchange> o_distExcange = std::make_shared<DistributionExchange>(DistributionExchange(configPath));

	int k = 1;
	size_t numParticles = 2000;
	size_t perCluster = numParticles / k;
	Eigen::Vector3f trans(1,1,0);

	std::vector<Eigen::Vector3f> centroids = {Eigen::Vector3f(0, 0, 0)};
	std::vector<Particle> particles(numParticles);
	for(int j = 0; j < k; ++j)
	{
		for (long unsigned int i = 0; i < perCluster; ++i)
		{
			particles[j * perCluster + i].pose = 0.1 * Eigen::Vector3f(drand48(), drand48(), drand48()) + centroids[j];
			particles[j * perCluster + i].weight = 1.0;
		}
	}

	std::shared_ptr<DistributionData> distData = o_distExcange->CreateDistribution(particles, DISTRIBUTION::PROROK, trans);

	std::vector<ClusterAbstraction> abstractions = distData->Prorok();
	ASSERT_EQ(abstractions.size(), 8);

	int numParticles2 = 2;
	std::vector<Particle> particles2(numParticles2);
	particles2[0].pose = trans;
	particles2[0].weight = 1.0;
	particles2[1].pose = Eigen::Vector3f(-10, 4, 0);
	particles2[1].weight = 1.0;

	o_distExcange->FuseDistribution(particles2, distData);

	ASSERT_NEAR(particles2[0].weight, 0.5, 0.05);
	ASSERT_NEAR(particles2[0].pose(0), trans(0), 0.05);
	ASSERT_NEAR(particles2[0].pose(1), trans(1), 0.05);
	ASSERT_NEAR(particles2[1].weight, 0.5, 0.05);
}




TEST(TestSetStatistics, test1)
{
	std::vector<Eigen::Vector3f> poses{Eigen::Vector3f(1,1,1), Eigen::Vector3f(1,1,1)};
	std::vector<double> weights {0.5, 0.5};
	Particle p1(Eigen::Vector3f(1,1,1), 0.5);
	Particle p2(Eigen::Vector3f(1,1,1), 0.5);
	std::vector<Particle> particles{p1, p2};


	SetStatistics stats = SetStatistics::ComputeParticleSetStatistics(particles);
	Eigen::Vector3d mean = stats.Mean();
	Eigen::Matrix3d cov = stats.Cov();

	ASSERT_EQ(mean, Eigen::Vector3d(1,1,1));

	ASSERT_NEAR(cov(0,0), 0, 0.000001);
	ASSERT_NEAR(cov(1,0), 0, 0.000001);
	ASSERT_NEAR(cov(0,1), 0, 0.000001);
	ASSERT_NEAR(cov(1,1), 0, 0.000001);
	ASSERT_NEAR(cov(2,2), 0, 0.000001);

}

TEST(TestSetStatistics, test2)
{
	std::vector<Eigen::Vector3f> poses{Eigen::Vector3f(1.3 ,1 ,1), Eigen::Vector3f(0.8, 0.7, 0)};
	std::vector<double> weights {0.5, 0.5};
	Particle p1(Eigen::Vector3f(1.3,1,1), 0.5);
	Particle p2(Eigen::Vector3f(0.8,0.7,0), 0.5);
	std::vector<Particle> particles{p1, p2};

	SetStatistics stats = SetStatistics::ComputeParticleSetStatistics(particles);
	Eigen::Vector3d mean = stats.Mean();
	Eigen::Matrix3d cov = stats.Cov();

	//ASSERT_EQ(mean, Eigen::Vector3d(1.05, 0.85, 0.5));
	ASSERT_NEAR(mean(0), 1.05 , 0.000001);
	ASSERT_NEAR(mean(1), 0.85 , 0.000001);
	ASSERT_NEAR(mean(2), 0.5 , 0.000001);
	ASSERT_NEAR(cov(0,0), 0.0625 , 0.000001);
	ASSERT_NEAR(cov(1,0), 0.0375 , 0.000001);
	ASSERT_NEAR(cov(0,1), 0.0375 , 0.000001);
	ASSERT_NEAR(cov(1,1), 0.0225, 0.000001);
	ASSERT_GE(cov(2,2), 0);  

}

TEST(TestSetStatistics, test3)
{
	std::vector<Eigen::Vector3f> poses{Eigen::Vector3f(1 ,1 ,1), Eigen::Vector3f(0, 0, 0)};
	std::vector<double> weights {1.0, 0.0};
	Particle p1(Eigen::Vector3f(1, 1,1 ), 1.0);
	Particle p2(Eigen::Vector3f(0, 0, 0), 0.0);
	std::vector<Particle> particles{p1, p2};

	SetStatistics stats = SetStatistics::ComputeParticleSetStatistics(particles);
	Eigen::Vector3d mean = stats.Mean();
	Eigen::Matrix3d cov = stats.Cov();

	ASSERT_EQ(mean, Eigen::Vector3d(1, 1, 1));

	ASSERT_NEAR(cov(0,0), 0, 0.000001);
	ASSERT_NEAR(cov(1,0), 0, 0.000001);
	ASSERT_NEAR(cov(0,1), 0, 0.000001);
	ASSERT_NEAR(cov(1,1), 0, 0.000001);
	ASSERT_NEAR(cov(2,2), 0, 0.000001);

}
/*
TEST(TestBeamEnd, test1)
{


	GMap gmap = GMap(dataPath);
	BeamEnd be = BeamEnd(std::make_shared<GMap>(gmap), 8, 15, BeamEnd::Weighting(0));

	Eigen::Vector3f p3d0_gt = Eigen::Vector3f(0.33675906, -0.84122932,  1. );
	std::vector<Eigen::Vector3f> scan{p3d0_gt};
	Particle p(Eigen::Vector3f(0, 0, 0), 1.0);
	std::vector<Particle> particles{p};
	std::vector<double> scanMask(1, 1.0);

	LidarData data = LidarData(scan, scanMask);

	be.ComputeWeights(particles, std::make_shared<LidarData>(data));

	//ASSERT_EQ(weights.size(), 1);

	// from python verified code weight should be 0.09063308, but the EDT is different therefore we expect some variance
	// Also sigma valued of BeamEnd was 8
	ASSERT_NEAR(particles[0].weight, 0.09063308, 0.01);
}

TEST(TestMCL, test1)
{

	float likelihoodSigma = 2;
	float maxRange = 20;
	std::shared_ptr<GMap> gmap = std::make_shared<GMap>(GMap(dataPath));
	std::shared_ptr<BeamEnd> sm = std::make_shared<BeamEnd>(gmap, likelihoodSigma, maxRange, BeamEnd::Weighting(1));
	std::shared_ptr<Resampling> rs = std::make_shared<Resampling>(Resampling());
	std::shared_ptr<FSR> mm = std::make_shared<FSR>(FSR());

	std::shared_ptr<MCL> mcl = std::make_shared<MCL>(MCL(gmap, mm, sm, rs, 1000));
	
}*/

TEST(TestMCL, test1)
{

	std::string configPath = configDir + "cmcl.config";
	std::cout << configPath << std::endl; 

	MCL mcl = MCL(configPath);
}

TEST(TestSampling, test1)
{
	int k = 5;
	std::vector<float> accW = {0.1, 0.3, 0.8, 0.9, 1.0};
	
	int c = 0;
	float u = 0.5;
	while(u > accW[c])
	{
		++c;
	}
	ASSERT_EQ(c, 2);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}