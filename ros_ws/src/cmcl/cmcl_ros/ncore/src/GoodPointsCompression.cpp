/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: GoodPointsCompression.cpp    			                   		       #
# ##############################################################################
**/

#include "GoodPointsCompression.h"
#include <pybind11/embed.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <iostream>

namespace py = pybind11;
using namespace py::literals;

GoodPointsCompression::GoodPointsCompression(const std::string& modulePath, Eigen::Vector2f detectionNoise)
{
	try {
		py::initialize_interpreter();
	}
	catch (...) {
	  std::cout << "Interpreter already initialized!" << std::endl;
	}
	py::module_ sys = py::module_::import("sys");
  py::list path = sys.attr("path");
  path.attr("append")(modulePath);

  //py::object gp = py::module_::import("goodpoints");
 	dc = py::module_::import("dist_compress");
  comp_kt = dc.attr("compresspp_kt_test");

  o_detectionNoise = detectionNoise;
  o_resampler = std::make_shared<Resampling>(Resampling());
}


Eigen::MatrixXi GoodPointsCompression::Compress(Eigen::MatrixXd& data)
{
	py::object result = comp_kt(data);
	Eigen::MatrixXi coreset = result.cast<Eigen::MatrixXi>();

	return coreset;
}




std::shared_ptr<DistributionData> GoodPointsCompression::Compress(const std::vector<Particle>& particles_, Eigen::Vector3f trans)
{
	std::vector<Particle> particles = computeDetectedParticles(particles_, trans);
	o_resampler->SetTH(10000000000); //always resample
	o_resampler->Resample(particles);

	int n = particles.size();
	Eigen::MatrixXd data(n ,2);

	 #pragma omp parallel for 
	for(int i = 0; i < n; ++i)
	{
	    data(i, 0) = particles[i].pose(0);
	    data(i, 1) = particles[i].pose(1);
	}


	py::object result = comp_kt(data);
	Eigen::MatrixXi c = result.cast<Eigen::MatrixXi>();
	
	int m = c.rows();
    std::vector<Particle> coreset(m);
    #pragma omp parallel for 
    for(int i = 0; i < m; ++i)
	{
		int index = c(i, 0);
		coreset[i] = particles[index];
	}

	float r = trans.head(2).norm();
	Eigen::Vector2f variance;
	variance(0) = r * o_detectionNoise(0);
	variance(1) = o_detectionNoise(1);

	//maybe compute the variance here somehow?

	std::shared_ptr<DistributionData> distData = std::make_shared<DistributionData>(DistributionData(DISTRIBUTION::GOODPOINTS, coreset, variance));

	return distData;
}

void GoodPointsCompression::Fuse(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData)
{
	std::vector<Particle> observerParticles = distData->Particles();
	Eigen::Vector2f variance = distData->Variance();

	int k = observerParticles.size();

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		double w = 0;
		Particle p = particles[i];
		Eigen::Vector3f pDet = p.pose;

		for(long unsigned int j = 0; j < k; ++j)
		{	
			Eigen::Vector3f pObs = observerParticles[j].pose;
			float dx = pObs(0) - pDet(0);
			float dy = pObs(1) - pDet(1);
			float dr =  dx * dx + dy * dy;
			float dt = Wrap2Pi(atan2(dy, dx));

			w += exp(-0.5 * (dr / variance(0)));// + (d_t * d_t) / variance(1)); //* observerParticles[j].weight;
			
		}

		// We need some weighing for wrong observation, like p(right detection)*p(r, t dist) + p(wrong detection)
		particles[i].weight *= (w + o_penalty);
	}

	//o_resampler->SetTH(0.5);
	//o_resampler->Resample(particles);
	ReciprocalSampling(particles, distData);
}

void GoodPointsCompression::ReciprocalSampling(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData)
{
	std::vector<Particle> observerParticles = distData->Particles();
	Eigen::Vector2f variance = distData->Variance();

	int k = observerParticles.size();

	o_resampler->SetTH(10000000000); //always resample
	o_resampler->Resample(particles);

	float dx = abs(variance(0) * cos(variance(1)));
	float dy = abs(variance(0) * sin(variance(1)));

	float alpha = 0.06;
	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		if(drand48() < alpha) 
		{
			int c = drand48() * k;
			particles[i] = observerParticles[c];
			particles[i].weight = 1.0 / particles.size();
			particles[i].pose(0) += SampleGuassian(sqrt(dx));
			particles[i].pose(1) += SampleGuassian(sqrt(dy));
			particles[i].pose(2) += Wrap2Pi(2 * M_PI * drand48() - M_PI);
		}
	}
}

