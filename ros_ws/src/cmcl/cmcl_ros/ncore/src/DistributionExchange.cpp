/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: DistributionExchange.cpp  				                               #
# ##############################################################################
**/

#include "DistributionExchange.h"
#include "Utils.h"
#include "KMeansClustering.h"
#include "ProrokClustering.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>
#include <exception>


DistributionExchange::DistributionExchange(const std::string& jsonConfigPath)
{
	using json = nlohmann::json;

	std::ifstream file(jsonConfigPath);
	json config;
	try
	{
		file >> config;
	}
	catch(std::exception& e)
	{
		std::cerr << "Failed to load config fail" << std::endl;
		std::cout << e.what() << '\n';
	}

	json detection = config["detectionModel"];
	o_penalty = detection["penalty"];
	detection["alpha"];
	o_detectionNoise = Eigen::Vector2f(detection["detectionNoise"]["x"], detection["detectionNoise"]["y"]);

	o_goodPoints = std::make_shared<GoodPointsCompression>(GoodPointsCompression("/ros_ws/src/cmcl/cmcl_ros/ncore/src/", o_detectionNoise));
	o_prorok = std::make_shared<ProrokClustering>(ProrokClustering(detection["prorok"]["clusters"], o_detectionNoise));
	o_det = std::make_shared<DensityEstimationTree>(DensityEstimationTree(detection["det"]["minSize"], detection["det"]["maxSize"]));
	o_kmeans = std::make_shared<KMeansClustering>(KMeansClustering(detection["kmeans"]["clusters"], detection["kmeans"]["iterations"], o_detectionNoise));
	o_thin = std::make_shared<StandardThinning>(StandardThinning(detection["thinning"]["clusters"], o_detectionNoise));
	//o_naive = std::make_shared<StandardThinning>(StandardThinning(detection["naive"]["clusters"], o_detectionNoise));
}


DistributionExchange::DistributionExchange(int k, Eigen::Vector2f detectionNoise, float penalty)
{

	o_penalty = penalty;
	o_k = k;
	o_detectionNoise = detectionNoise;

	o_goodPoints = std::make_shared<GoodPointsCompression>(GoodPointsCompression("/ros_ws/src/cmcl/cmcl_ros/ncore/src/", o_detectionNoise));
	o_prorok = std::make_shared<ProrokClustering>(ProrokClustering(o_k, o_detectionNoise));
	o_det = std::make_shared<DensityEstimationTree>(DensityEstimationTree(500, 1000));
	o_kmeans = std::make_shared<KMeansClustering>(KMeansClustering(o_k, 5, detectionNoise));
	o_thin = std::make_shared<StandardThinning>(StandardThinning(o_k, detectionNoise));
}

std::shared_ptr<DistributionData> DistributionExchange::CreateDistribution(const std::vector<Particle>& detectedParticles, DISTRIBUTION distType, Eigen::Vector3f trans)
{

	std::shared_ptr<DistributionData> distData;

	switch(distType)
	{
		case DISTRIBUTION::PARTICLES:
	    {
	    	distData = o_thin->Compress(detectedParticles, trans);
	    	break;
	    }
	    case DISTRIBUTION::DET:
	    {
	    	distData = o_det->Compress(detectedParticles, trans);
	    	break;
	    }
		case DISTRIBUTION::KMEANS:
	    {
	    	distData = o_kmeans->Compress(detectedParticles, trans);
	    	break;
	    }
		case DISTRIBUTION::PROROK:
		{
			 distData = o_prorok->Compress(detectedParticles, trans);
			 break;
		}
		case DISTRIBUTION::GOODPOINTS:
		{
			distData = o_goodPoints->Compress(detectedParticles, trans);
			break;
		}
	}

	return distData;
}


void DistributionExchange::FuseDistribution(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData)
{
	DISTRIBUTION distType = distData->Type();

	switch(distType)
	{
		case DISTRIBUTION::PARTICLES:
	    {
	    	o_thin->Fuse(particles, distData);
	    	break;
	    }
	    case DISTRIBUTION::DET:
	    {
	    	o_det->Fuse(particles, distData);
	    	break;
	    }
	    case DISTRIBUTION::KMEANS:
	    {
	    	o_kmeans->Fuse(particles, distData);
	    	break;
	    }
		case DISTRIBUTION::PROROK:
		{
			o_prorok->Fuse(particles, distData);
			break;
		}
		case DISTRIBUTION::GOODPOINTS:
		{
			o_goodPoints->Fuse(particles, distData);
			break;
		}
	}

}



/*
void DistributionExchange::Naive(std::vector<Particle>& particles, const std::vector<Particle>& observerParticles, const Eigen::Vector3f& relTrans)
{
	std::vector<Particle> detectedParticles = std::vector<Particle>(observerParticles.size());

	Eigen::Vector3f relTransXY(relTrans(0), relTrans(1), 1.0);

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < observerParticles.size(); ++i)
	{
			Eigen::Matrix3f particleFrame = Vec2Trans(observerParticles[i].pose);
			Eigen::Vector3f top = particleFrame * relTransXY;
			top(2) =  relTrans(2) + observerParticles[i].pose(2);
			detectedParticles[i].pose = top;
			detectedParticles[i].weight = observerParticles[i].weight;
	}

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		double w = 0;
		Particle p = particles[i];
		float yaw = p.pose(2);

		for(long unsigned int i = 0; i < detectedParticles.size(); ++i)
		{	
			Particle pDet = detectedParticles[i];
			float diff_r = (p.pose(0) - pDet.pose(0)) * (p.pose(0) - pDet.pose(0)) + (p.pose(1) - pDet.pose(1)) * (p.pose(1) - pDet.pose(1));
			float dist_r = exp(-0.5 * (diff_r / o_sigma_r_sq));

			float yawDet = pDet.pose(2);
			float sim = CosineSimilarity(yawDet, yaw);
			float diff_t = 1.0 - sim;
			//Particle pObs = observerParticles[i];
			//float diff_t = atan2(p.pose(1) - pObs.pose(1), p.pose(0) - pObs.pose(0)) - relTrans(2);
			float dist_t = exp(-0.5 * (diff_t / o_sigma_t_sq));
			w += pDet.weight * dist_t * dist_r;
		}

		// We need some weighing for wrong observation, like p(right detection)*p(r, t dist) + p(wrong detection)
		particles[i].weight *= (w + o_penalty);
	}
}


void DistributionExchange::Naive(std::vector<Particle>& particles, const std::vector<Particle>& observerParticles, const Eigen::Vector3f& relTrans)
{

	float r = sqrt(relTrans(0) * relTrans(0) + relTrans(1) * relTrans(1));
	float b = atan2(relTrans(1), relTrans(0));

	
	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		double w = 0;
		Particle p = particles[i];
		Eigen::Vector3f pDet = p.pose;

		for(long unsigned int j = 0; j < observerParticles.size(); ++j)
		{	
			Eigen::Vector3f pObs = observerParticles[j].pose;
			float d_x = pObs(0) - pDet(0);
			float d_y = pObs(1) - pDet(1);
			float d_r =  sqrt(d_x * d_x + d_y * d_y) - r;
			//float d_t = atan2(d_y, d_x) - (pObs(2) + b);
			float sim = CosineSimilarity(atan2(d_y, d_x), (pObs(2) + b) + M_PI);
			float d_t = 1.0 - sim;

			w += observerParticles[j].weight * exp(-0.5 * ((d_t) / 0.1)) * exp(-0.5 * ((d_r * d_r) / o_sigma_r_sq));
			
		}

		// We need some weighing for wrong observation, like p(right detection)*p(r, t dist) + p(wrong detection)
		particles[i].weight *= (w + o_penalty);
	}
}


std::vector<Particle> DistributionExchange::resample(const std::vector<Particle>& particles)
{
	int n_particles = particles.size();
	std::vector<Particle> new_particles(n_particles);

	double unitW = 1.0 / n_particles;
	//std::cout << "resample" << std::endl;
	double r = drand48() * 1.0 / n_particles;
	double acc = particles[0].weight;
	int i = 0;

	for(int j = 0; j < n_particles; ++j)
	{
		double U = r + j * 1.0 / n_particles;
		while((U > acc) && (i < n_particles))
		{
			++i;
			acc += particles[i].weight;
		}
		new_particles[j] = particles[i];
		new_particles[i].weight = unitW;
	}
	return new_particles;
	
}

void DistributionExchange::resampleInPlace(std::vector<Particle>& particles)
{
	int n_particles = particles.size();
	std::vector<Particle> new_particles(n_particles);

	double unitW = 1.0 / n_particles;
	//std::cout << "resample" << std::endl;
	double r = drand48() * 1.0 / n_particles;
	double acc = particles[0].weight;
	int i = 0;

	for(int j = 0; j < n_particles; ++j)
	{
		double U = r + j * 1.0 / n_particles;
		while((U > acc) && (i < n_particles))
		{
			++i;
			acc += particles[i].weight;
		}
		particles[i].weight = unitW;
		new_particles[j] = particles[i];
	}
	particles = new_particles;

	
}


std::vector<Particle> resampleSpecial(const std::vector<Particle>& particles)
{
	int n_particles = particles.size();
	std::vector<double> transWeights(n_particles);
	std::vector<Particle> new_particles(n_particles);

	double unitW = 1.0 / n_particles;
	//std::cout << "resample" << std::endl;
	double r = drand48() * 1.0 / n_particles;
	double acc = particles[0].weight;
	int i = 0;

	for(int j = 0; j < n_particles; ++j)
	{
		double U = r + j * 1.0 / n_particles;
		while((U > acc) && (i < n_particles))
		{
			++i;
			acc += particles[i].weight;
		}
		new_particles[j] = particles[i];
		new_particles[i].weight = unitW;
		new_particles[i].pose(2) = drand48() * 2 * M_PI - M_PI;
	}
	return new_particles;
}


void DistributionExchange::DET(std::vector<Particle>& particles, const std::vector<Particle>& observerParticles, const Eigen::Vector3f& relTrans)
{
	size_t numParticles = observerParticles.size();
	Eigen::Vector3f transVec = relTrans;
	transVec(2) = 0.0;
	Eigen::Matrix3f  trans = Vec2Trans(transVec).inverse();

	std::vector<Particle> resampledObserverParticles = resample(observerParticles);

	//  std::vector<Particle>::const_iterator it = std::min_element(observerParticles.begin(), observerParticles.end(),
    //         [] (Particle const & lhs, Particle const & rhs) {
    //         return lhs.weight < rhs.weight;
    // });

	//  double minWeight = (*it).weight;

	//  std::vector<Eigen::Vector2f> data;
	//  //#pragma omp parallel for 
	// for (long unsigned int i = 0; i < numParticles; ++i)
	// {
	// 	 double w = observerParticles[i].weight;
	// 	int count = int(w / minWeight) + 1;
	// 	for (long unsigned int j = 0; j < count; ++j)
	// 	{
	// 		data.push_back(Eigen::Vector2f(observerParticles[i].pose(0), observerParticles[i].pose(1)));
	// 	}

	// }
	// size_t numData = data.size();
	// arma::mat testData(2, numData);
	// arma::Col<size_t> oTest(numData);

	// #pragma omp parallel for 
	// for (long unsigned int i = 0; i < numData; ++i)
	// {
	// 	testData(0, i) = data[i](0);
	// 	testData(1, i) = data[i](1);
	// 	oTest(i) = i;
	// }

	arma::mat testData(2, numParticles);
	arma::Col<size_t> oTest(numParticles);

	#pragma omp parallel for 
	for (long unsigned int i = 0; i < numParticles; ++i)
	{
		testData(0, i) = resampledObserverParticles[i].pose(0);
		testData(1, i) = resampledObserverParticles[i].pose(1);
		oTest(i) = i;
	}

	DTree<arma::mat> testDTree(testData);
	double alpha = testDTree.Grow(testData, oTest, false, 10, 5);

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		Eigen::Vector3f p = particles[i].pose;
		p(2) = 1.0;
		Eigen::Vector3f top = trans * p;
		arma::vec q(2);
		q = { top(0), top(1) };
		double w = (testDTree.ComputeValue(q)) / numParticles;
		particles[i].weight *= (w + o_penalty);
	}

}

void DistributionExchange::DET(std::vector<Particle>& particles, const std::vector<Particle>& observerParticles, const Eigen::Vector3f& relTrans)
{

	std::vector<Particle> detectedParticles = std::vector<Particle>(observerParticles.size());

	Eigen::Vector3f relTransXY(relTrans(0), relTrans(1), 1.0);

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < observerParticles.size(); ++i)
	{
			Eigen::Matrix3f particleFrame = Vec2Trans(observerParticles[i].pose);
			Eigen::Vector3f top = particleFrame * relTransXY;
			top(2) =  relTrans(2) + observerParticles[i].pose(2);
			detectedParticles[i].pose = top;
			detectedParticles[i].weight = observerParticles[i].weight;
	}



	size_t numParticles = detectedParticles.size();
	std::vector<Particle> resampledObserverParticles = resample(detectedParticles);


	arma::mat testData(2, numParticles);
	arma::Col<size_t> oTest(numParticles);

	#pragma omp parallel for 
	for (long unsigned int i = 0; i < numParticles; ++i)
	{
		testData(0, i) = resampledObserverParticles[i].pose(0);
		testData(1, i) = resampledObserverParticles[i].pose(1);
		oTest(i) = i;
	} 

	DTree<arma::mat> testDTree(testData);
	double alpha = testDTree.Grow(testData, oTest, false, 1000, 500);

	// no prunning, can add https://github.com/mlpack/mlpack/blob/master/src/mlpack/methods/det/dt_utils_impl.hpp


	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		Eigen::Vector3f p = particles[i].pose;
		arma::vec q(2);
		q = { p(0), p(1) };
		double w = (testDTree.ComputeValue(q)) ;
		particles[i].weight *= (w + o_penalty);
	}

}
*/
