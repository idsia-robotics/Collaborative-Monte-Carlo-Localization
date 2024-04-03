/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: MCL.cpp                    			                               #
# ##############################################################################
**/


#include "MCL.h"
#include <numeric>
#include <functional>  
#include <iostream>
#include <fstream>
#include "Utils.h"
#include <algorithm>
#include <iterator>
#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>
#include <exception>


MCL::MCL(const std::string& jsonConfigPath)
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

	o_motionModel = std::make_shared<FSR>(FSR());
	o_resampler = std::make_shared<Resampling>(Resampling());

	std::string folderPath = boost::filesystem::path(jsonConfigPath).parent_path().string() + "/";
	std::string mapPath = std::string(config["mapPath"]);
	o_gmap = std::make_shared<GMap>(GMap(folderPath, mapPath));

	float likelihoodSigma = config["sensorModel"]["likelihoodSigma"];
	float maxDist = config["sensorModel"]["maxDist"];
	int wScheme = config["sensorModel"]["weightingScheme"];
	o_beamEndModel = std::make_shared<BeamEnd>(o_gmap, likelihoodSigma, maxDist, BeamEnd::Weighting(1));

	int clusterNum = config["detectionModel"]["clusters"];
	DISTRIBUTION distType = static_cast<DISTRIBUTION>(config["detectionModel"]["type"]);
	//o_distExcange = std::make_shared<DistributionExchange>(DistributionExchange(clusterNum, o_detectionNoise));
	o_distExcange = std::make_shared<DistributionExchange>(DistributionExchange(jsonConfigPath));

	o_particleFilter = std::make_shared<ParticleFilter>(ParticleFilter(o_gmap));
	o_numParticles = config["numParticles"];
	bool tracking = config["tracking"]["mode"];
	if (tracking)
	{
		std::vector<Eigen::Matrix3d> covariances;
		std::vector<Eigen::Vector3f> initGuesses;

		std::vector<float> x = config["tracking"]["x"];
		std::vector<float> y = config["tracking"]["y"];
		std::vector<float> yaw = config["tracking"]["yaw"];
		std::vector<float> covx = config["tracking"]["cov_x"];
		std::vector<float> covy = config["tracking"]["cov_y"];
		std::vector<float> covyaw = config["tracking"]["cov_yaw"];

		for (int i = 0; i < x.size(); ++i)
		{
			Eigen::Matrix3d cov;
			cov << covx[i], 0, 0, 0, covy[i], 0, 0, 0, covyaw[i]; 
			covariances.push_back(cov);
			initGuesses.push_back(Eigen::Vector3f(x[i], y[i], yaw[i]));
		}
		o_particleFilter->InitGaussian(o_particles, o_numParticles, initGuesses, covariances);
	}
	else o_particleFilter->InitUniform(o_particles, o_numParticles);

	o_stats = SetStatistics::ComputeParticleSetStatistics(o_particles);
}

MCL::MCL(std::shared_ptr<GMap> gmap, std::shared_ptr<FSR> mm, std::shared_ptr<BeamEnd> sm, 
			std::shared_ptr<Resampling> rs, int n_particles)
{
	o_motionModel = mm;
	o_beamEndModel = sm;
	o_resampler = rs;
	o_numParticles = n_particles;
	o_gmap = gmap;
	o_detectionNoise = Eigen::Vector2f(0.1, 0.1);
	
	o_distExcange = std::make_shared<DistributionExchange>(DistributionExchange(8, o_detectionNoise));
	o_particleFilter = std::make_shared<ParticleFilter>(ParticleFilter(o_gmap));
	o_particleFilter->InitUniform(o_particles, o_numParticles);
	o_stats = SetStatistics::ComputeParticleSetStatistics(o_particles);

	//dumpParticles();

}

MCL::MCL(std::shared_ptr<GMap> gmap, std::shared_ptr<FSR> mm, std::shared_ptr<BeamEnd> sm, 
			std::shared_ptr<Resampling> rs, int n_particles, std::vector<Eigen::Vector3f> initGuess, 
			std::vector<Eigen::Matrix3d> covariances)
{
	o_motionModel = mm;
	o_beamEndModel = sm;
	o_resampler = rs;
	o_numParticles = n_particles;
	o_gmap = gmap;
	
	o_particleFilter = std::make_shared<ParticleFilter>(ParticleFilter(o_gmap));
	o_particleFilter->InitGaussian(o_particles, o_numParticles, initGuess, covariances);
	o_stats = o_particleFilter->ComputeStatistics(o_particles);
	
}


void MCL::dumpParticles()
{
	std::string path =  std::to_string(o_frame) + "_particles.csv";
	std::ofstream particleFile;
    particleFile.open(path, std::ofstream::out);
    particleFile << "x" << "," << "y" << "," << "yaw" << "," << "w" << std::endl;
    for(long unsigned int p = 0; p < o_particles.size(); ++p)
    {
        Eigen::Vector3f pose = o_particles[p].pose;
        float w = o_particles[p].weight;
        particleFile << pose(0) << "," << pose(1) << "," << pose(2) << "," << w << std::endl;
    }
    particleFile.close();
    ++o_frame;
}



void MCL::Predict(const std::vector<Eigen::Vector3f>& u, const std::vector<float>& odomWeights, const Eigen::Vector3f& noise)
{
	#pragma omp parallel for 
	for(int i = 0; i < o_numParticles; ++i)
	{
		Eigen::Vector3f pose = o_motionModel->SampleMotion(o_particles[i].pose, u, odomWeights, noise);
		o_particles[i].pose = pose;

		// if (drand48() < 0.01)
		// {
		// 	std::vector<Particle> new_particle;
		// 	o_particleFilter->InitUniform(new_particle, 1);
		// 	new_particle[0].weight = 1.0 / o_numParticles;
		// 	o_particles[i] = new_particle[0];
		// }
		
		//particle pruning - if particle is outside the map, we replace it
		while (!o_gmap->IsValid(o_particles[i].pose))
		{
			std::vector<Particle> new_particle;
			o_particleFilter->InitUniform(new_particle, 1);
			new_particle[0].weight = 1.0 / o_numParticles;
			o_particles[i] = new_particle[0];
		}
	}
}





void MCL::Correct(std::shared_ptr<LidarData> data)
{
	o_beamEndModel->ComputeWeights(o_particles, data);
	//dumpParticles();

	//o_particleFilter->NormalizeWeights(o_particles);
	o_resampler->Resample(o_particles);
	//o_stats = SetStatistics::ComputeParticleSetStatistics(o_particles);
	o_stats = o_particleFilter->ComputeStatistics(o_particles);
	//std::cout << o_stats.Mean() << std::endl;

}


void MCL::Recover()
{
	o_particleFilter->InitUniform(o_particles, o_numParticles);
}


int MCL::traceInMap(Eigen::Vector3f p1, Eigen::Vector3f p2)
{

	Eigen::Vector2f uv1 = o_gmap->World2Map(p1.head(2));
	Eigen::Vector2f uv2 = o_gmap->World2Map(p2.head(2));

	if (false == o_gmap->IsValid2D(uv2)) return -1;

	float dist = (uv2 - uv1).norm();
	Eigen::Vector2f dir = (uv2 - uv1).normalized();

	Eigen::Vector2f currPose = uv1;
	float step = 1;
	int count = 0;

	for(int i = 0; i < int(dist / step); ++i)
	{
		currPose += step * dir;
		if (!o_gmap->IsValid2D(currPose))
		{
			++count;
		}
	}

	return count;
}


void MCL::detectionReweight(Eigen::Vector3f trans)
{
	std::vector<Particle> shiftedParticles = computeDetectedParticles(o_particles, trans);
	float res = o_gmap->Resolution();
	float r = trans.head(2).norm();
	float sigma = r * o_detectionNoise(0);
	float maxDist = 20;
	int numParticles = o_particles.size();

	#pragma omp parallel for 
	for (long unsigned int i = 0; i < numParticles; ++i)
	{
			int trace = traceInMap(o_particles[i].pose, shiftedParticles[i].pose);
			if (trace < 0) o_particles[i].weight *= exp(-0.5 *(maxDist * maxDist)/ sigma);
			else
			{
				o_particles[i].weight *= exp(-0.5 * (trace * res) * (trace * res) / sigma);
			}

	}

	o_particleFilter->NormalizeWeights(o_particles);
}
 

#include <fstream>
std::shared_ptr<DistributionData> MCL::CreateDistribution(DISTRIBUTION distType, const Eigen::Vector3f& relTrans)
{

	/*if ((distType != DISTRIBUTION::DET) and (distType != DISTRIBUTION::PROROK))
	{
		detectionReweight(relTrans);
	}*/
	std::vector<Particle> detectedParticles;
	std::copy(o_particles.begin(), o_particles.end(), std::back_inserter(detectedParticles));
	

	std::shared_ptr<DistributionData> distData = o_distExcange->CreateDistribution(detectedParticles, distType, relTrans);
	o_stats = o_particleFilter->ComputeStatistics(o_particles);


	return distData;	
}

void MCL::FuseDistribution(std::shared_ptr<DistributionData>& distData)
{
	DISTRIBUTION distType = distData->Type();
	Eigen::Vector2f variance = distData->Variance();
	/*switch(distType) 
    {
        case DISTRIBUTION::PARTICLES:
        {
        	if ((variance(0) > 2.0) || (variance(1) > 2.0)) return;
        	break;
        }
        case DISTRIBUTION::DET:
        {
        	if ((variance(0) > 2.0) || (variance(1) > 2.0)) return;
        	break;
        }
        case DISTRIBUTION::KMEANS:
        {
        	bool ignore = true;
        	std::vector<Eigen::Vector2f> variances = distData->KMeans()->variances;
        	for(long unsigned int i = 0; i < variances.size(); ++i)
        	{
        		Eigen::Vector2f variance = variances[i];
        		if ((variance(0) < 2.0) && (variance(1) < 2.0)) ignore = false;
        	}
        	if (ignore) return;
        	break;
        }
    }*/
    /*static int countB = 0;
	using namespace std;
	ofstream myfile;
	myfile.open ("particlesB_" + std::to_string(countB) + ".csv", std::ios_base::app);
	++countB;

	for(long unsigned int i = 0; i < o_particles.size(); ++i)
	{
		Eigen::Vector3f p  = o_particles[i].pose;
		myfile << p(0) << ", "  << p(1) << ", "  << p(2) << ", " << o_particles[i]. weight << std::endl;
	}
	myfile.close();*/

	o_distExcange->FuseDistribution(o_particles, distData);

	//o_resampler->Resample(o_particles);
	//o_stats = o_particleFilter->ComputeStatistics(o_particles);
}




