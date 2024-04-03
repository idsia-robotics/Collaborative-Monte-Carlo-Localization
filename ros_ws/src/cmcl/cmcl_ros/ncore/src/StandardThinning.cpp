/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: StandardThinning.cpp  		    		                               #
# ##############################################################################
**/

#include "StandardThinning.h"



StandardThinning::StandardThinning(int subSize, Eigen::Vector2f detectionNoise)
{
	o_n = subSize;
	o_detectionNoise = detectionNoise;  
	o_resampler = std::make_shared<Resampling>(Resampling());
}

std::shared_ptr<DistributionData> StandardThinning::Compress(const std::vector<Particle>& particles_, Eigen::Vector3f trans)
{
	std::vector<Particle> particles = computeDetectedParticles(particles_, trans);

	float r = trans.head(2).norm();
	Eigen::Vector2f variance;
	variance(0) = r * o_detectionNoise(0);
	variance(1) = o_detectionNoise(1);
	std::shared_ptr<DistributionData> distData;
	int m = particles.size();

	if (m == o_n)
	{
		distData = std::make_shared<DistributionData>(DistributionData(DISTRIBUTION::PARTICLES, particles, variance));
	}
	else
	{
		std::vector<Particle> subset(o_n);
		for(int j = 0; j < o_n; ++j)
		{
			subset[j] = particles[(int)(drand48() * m)];
		}
		distData = std::make_shared<DistributionData>(DistributionData(DISTRIBUTION::PARTICLES, subset, variance));
	}

	return distData;
}

void StandardThinning::Fuse(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData)
{
	std::vector<Particle>& observerParticles = distData->Particles();
	Eigen::Vector2f variance = distData->Variance();

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		double w = 0;
		Particle p = particles[i];
		Eigen::Vector3f pDet = p.pose;

		for(long unsigned int j = 0; j < observerParticles.size(); ++j)
		{	
			Eigen::Vector3f pObs = observerParticles[j].pose;
			float dx = pObs(0) - pDet(0);
			float dy = pObs(1) - pDet(1);
			float dr =  dx * dx + dy * dy;
			float dt = Wrap2Pi(atan2(dy, dx));

			w += exp(-0.5 * (dr / variance(0))) * observerParticles[j].weight;
		}

		// We need some weighing for wrong observation, like p(right detection)*p(r, t dist) + p(wrong detection)
		particles[i].weight *= (w + o_penalty);
	}

	//o_resampler->SetTH(0.5); 
	//o_resampler->Resample(particles);
	ReciprocalSampling(particles, distData);

}

void StandardThinning::ReciprocalSampling(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData)
{
	std::vector<Particle>& observerParticles = distData->Particles();
	Eigen::Vector2f variance = distData->Variance();

	int m = observerParticles.size();
	o_resampler->SetTH(10000000000); //always resample
	o_resampler->Resample(observerParticles);
	o_resampler->Resample(particles);
	float dx = abs(variance(0) * cos(variance(1)));
	float dy = abs(variance(0) * sin(variance(1)));

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		if(drand48() < 0.06) 
		{
			particles[i] = observerParticles[(int)(drand48() * m)];
			particles[i].pose(0) += SampleGuassian(sqrt(dx));
			particles[i].pose(1) += SampleGuassian(sqrt(dy));
			particles[i].pose(2) += 2 * M_PI * drand48() - M_PI;
			particles[i].weight = 1.0 / particles.size();
		}
	}

}

