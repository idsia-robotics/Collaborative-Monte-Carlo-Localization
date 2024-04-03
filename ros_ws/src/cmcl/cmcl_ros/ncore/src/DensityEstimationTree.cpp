/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: DensityEstimationTree.cpp                                             #
# ##############################################################################
**/

#include "DensityEstimationTree.h"

#include <fstream>
#include <mlpack.hpp>
#include <mlpack/methods/det/dt_utils.hpp>
#include <mlpack/methods/det.hpp>
using namespace mlpack;
using namespace std;



DensityEstimationTree::DensityEstimationTree(int minLeafSize, int maxLeafSize)
{
	o_minLeafSize = minLeafSize;
	o_maxLeafSize = maxLeafSize;
	o_resampler = std::make_shared<Resampling>(Resampling());
}

std::shared_ptr<DTree<arma::mat>> DensityEstimationTree::TrainDET(const std::vector<Particle>& particles)
{
	size_t numParticles = particles.size();
	arma::mat testData(2, numParticles);
	arma::Col<size_t> oTest(numParticles);

	#pragma omp parallel for 
	for (long unsigned int i = 0; i < numParticles; ++i)
	{
		testData(0, i) = particles[i].pose(0);
		testData(1, i) = particles[i].pose(1);
		oTest(i) = i;
	} 

	std::shared_ptr<DTree<arma::mat>> testDTree = std::make_shared<DTree<arma::mat>>(DTree<arma::mat>(testData));
	double alpha = testDTree->Grow(testData, oTest, false, o_maxLeafSize, o_minLeafSize);
	
	return testDTree;
}

double DensityEstimationTree::QueryDET(std::shared_ptr<DTree<arma::mat>> tree, Particle p)
{
	arma::vec q(2);
	q = { p.pose(0), p.pose(1) };
	double w = (tree->ComputeValue(q)) ;

	return w;
}


std::shared_ptr<DistributionData> DensityEstimationTree::Compress(const std::vector<Particle>& particles_, Eigen::Vector3f trans)
{	
	std::vector<Particle> particles = computeDetectedParticles(particles_, trans);
	o_resampler->SetTH(10000000000); //always resample
	o_resampler->Resample(particles);

	size_t numParticles = particles.size();
	arma::mat testData(2, numParticles);
	arma::Col<size_t> oTest(numParticles);

	#pragma omp parallel for 
	for (long unsigned int i = 0; i < numParticles; ++i)
	{
		testData(0, i) = particles[i].pose(0);
		testData(1, i) = particles[i].pose(1);
		oTest(i) = i;
	} 

	DTree<arma::mat> testDTree(testData);
	double alpha = testDTree.Grow(testData, oTest, false, o_maxLeafSize, o_minLeafSize);

	// no prunning, can add https://github.com/mlpack/mlpack/blob/master/src/mlpack/methods/det/dt_utils_impl.hpp

	std::ostringstream os(std::stringstream::binary);
	cereal::BinaryOutputArchive arout(os);
	arout(cereal::make_nvp("det", testDTree));
	std::string osstr = os.str();
	std::vector<uint8_t> bytes(osstr.begin(), osstr.end());
	std::shared_ptr<DistributionData> distData = std::make_shared<DistributionData>(DistributionData(DISTRIBUTION::DET, bytes));

	return distData;
}

void DensityEstimationTree::Fuse(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData)
{
	std::vector<uint8_t> distributionBytes = distData->DET();
	DTree<arma::mat> testDTree;

	std::string distribution(distributionBytes.begin(),distributionBytes.end());

	std::istringstream is(distribution, std::stringstream::binary);
	cereal::BinaryInputArchive arin(is);
	try {
        arin(cereal::make_nvp("det", testDTree));
    }
    catch (std::runtime_error e) {

    	#include <fstream>
		using namespace std;
		ofstream myfile;
		myfile.open ("exception.txt");
		myfile << e.what() << std::endl;
		myfile.close();
        e.what();
    }

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		Eigen::Vector3f p = particles[i].pose;
		arma::vec q(2);
		q = { p(0), p(1) };
		double w = (testDTree.ComputeValue(q)) ;
		particles[i].weight *= (w + o_penalty);
		//std::cout << w << std::endl;
	}

	o_resampler->SetTH(0.5);
	o_resampler->Resample(particles);
}
