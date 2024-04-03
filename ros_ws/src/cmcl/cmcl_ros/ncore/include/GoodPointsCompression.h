/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: GoodPointsCompression.h      			                   		       #
# ##############################################################################
**/


#ifndef GOODPOINTSCOMPRESSION_H
#define GOODPOINTSCOMPRESSION_H

#include <eigen3/Eigen/Dense>
#include <pybind11/embed.h>
#include "Particle.h"
#include "DistributionData.h"
#include "Resampling.h"

class GoodPointsCompression 
{

	public: 
		GoodPointsCompression(const std::string& modulePath, Eigen::Vector2f detectionNoise = Eigen::Vector2f(0.15, 0.15));
		

		std::shared_ptr<DistributionData> Compress(const std::vector<Particle>& particles, Eigen::Vector3f trans);

		Eigen::MatrixXi Compress(Eigen::MatrixXd& data);


		void Fuse(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);

		void ReciprocalSampling(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);



	private:

		pybind11::object comp_kt;
		pybind11::object dc;
		float o_penalty = 0.000000001; //0.0000000000000001; 
		Eigen::Vector2f o_detectionNoise;
		std::shared_ptr<Resampling> o_resampler; 
};

#endif