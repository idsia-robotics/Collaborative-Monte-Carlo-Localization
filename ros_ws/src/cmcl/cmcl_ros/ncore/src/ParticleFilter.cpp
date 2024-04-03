/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: ParticleFilter.cpp           		                           		   #
# ##############################################################################
**/
 
 #include "ParticleFilter.h"
#include "Utils.h"
#include <algorithm>
#include <map>

ParticleFilter::ParticleFilter(std::shared_ptr<GMap> GMap)
{
	o_gmap = GMap;
}




void ParticleFilter::InitUniform(std::vector<Particle>& particles, int n_particles)
{
	particles = std::vector<Particle>(n_particles);

	Eigen::Vector2f tl = o_gmap->Map2World(o_gmap->TopLeft());
	Eigen::Vector2f br = o_gmap->Map2World(o_gmap->BottomRight());

	int i = 0;
	while(i < n_particles)
	{
			float x = drand48() * (br(0) - tl(0)) + tl(0);
			float y = drand48() * (br(1) - tl(1)) + tl(1);
			if(!o_gmap->IsValid(Eigen::Vector3f(x, y, 0))) continue;

			float theta = drand48() * 2 * M_PI - M_PI;
			Particle p(Eigen::Vector3f(x, y, theta), 1.0 / n_particles);
			particles[i] = p;
			++i;
	}

	//particles[0] = Particle(Eigen::Vector3f(0.0, 0.0, 0.0), 1.0 / n_particles);
	

	o_particles = particles;
}


void ParticleFilter::InitGaussian(std::vector<Particle>& particles, int n_particles, const std::vector<Eigen::Vector3f>& initGuess, const std::vector<Eigen::Matrix3d>& covariances)
{
	particles = std::vector<Particle>(n_particles);

	for(long unsigned int i = 0; i < initGuess.size(); ++i)
	{
		Eigen::Vector3f initG = initGuess[i].head(3);
		Eigen::Matrix3d cov = covariances[i];

		float dx = fabs(SampleGuassian(cov(0, 0)));
		float dy = fabs(SampleGuassian(cov(1, 1)));
		float dt = fabs(SampleGuassian(cov(2, 2)));

		if (cov(2, 2) < 0.0) dt = M_PI;

		Eigen::Vector3f delta(dx, dy, dt);

		Eigen::Vector3f tl = initG + delta;
		Eigen::Vector3f br = initG - delta;

		int n = 0;
		while(n < n_particles)
		{
				float x = drand48() * (br(0) - tl(0)) + tl(0);
				float y = drand48() * (br(1) - tl(1)) + tl(1);
				if(!o_gmap->IsValid(Eigen::Vector3f(x, y, 0))) continue;

				float theta = drand48() * (br(2) - tl(2)) + tl(2);
				Particle p(Eigen::Vector3f(x, y, theta), 1.0 / n_particles);
				particles[n + n_particles * i] = p;
				++n;
		}
	}

	o_particles = particles;
}




SetStatistics ParticleFilter::ComputeStatistics(const std::vector<Particle>& particles)
{
	o_stats = SetStatistics::ComputeParticleSetStatistics(o_particles);

	return o_stats;
}




void ParticleFilter::NormalizeWeights(std::vector<Particle>& particles)
{
	
	double w = 0;
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		w += particles[i].weight;
	}

	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		particles[i].weight = particles[i].weight / w;
	}
}
