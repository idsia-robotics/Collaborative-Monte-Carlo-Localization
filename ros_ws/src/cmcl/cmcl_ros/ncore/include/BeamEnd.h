/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: BeamEnd.h                      		                               #
# ##############################################################################
**/


#ifndef BEAMEND_H
#define BEAMEND_H

#include <memory>
#include <string>
#include "GMap.h"
#include <vector>
#include <eigen3/Eigen/Dense>
#include "LidarData.h"
#include "Particle.h"
#include "GMap.h"

class BeamEnd
{
	public:

		enum class Weighting 
		{   
			NAIVE = 0, 
		    GEOMETRIC = 1
		};


		//! A constructor
	    /*!
	      \param Gmap is a ptr to a GMAP object, which holds the gmapping map
	      \param sigma is a float that determine how forgiving the model is (small sigma will give a very peaked likelihood)
	      \param maxDist is a float specifying up to what distance from the sensor a reading is valid
	      \param Weighting is an int specifying which weighting scheme to use
	    */

		BeamEnd(const std::shared_ptr<GMap>& GMap, float sigma = 8, float maxDist = 15, Weighting weighting = Weighting::GEOMETRIC);

		//! Computes weights for all particles based on how well the observation matches the map
		/*!
		  \param particles is a vector of Particle elements
		  \param SensorData is an abstract container for sensor data. This function expects LidarData type
		*/

		void ComputeWeights(std::vector<Particle>& particles, std::shared_ptr<LidarData> data);
		
		//! Returns truth if a particle is in an occupied grid cell, false otherwise. Notice that for particles in unknown areas the return is false.
		/*!
		  \param Particle is a particle with pose and weight
		*/


		//void plotParticles(std::vector<Particle>& particles, std::string title, bool show=true); 

		const cv::Mat& DebugMap()
		{
			return o_debugMap;
		}
	
	
	private:	


		float getLikelihood(float distance);

		void plotScan(Eigen::Vector3f laser, std::vector<Eigen::Vector2f>& zMap); 

		std::vector<Eigen::Vector2f> scan2Map(const Particle& particle, const std::vector<Eigen::Vector3f>& scan);

		double naive(const Particle& particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask);

		double geometric(const Particle& particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask);


		std::shared_ptr<GMap> o_gmap;
		cv::Mat o_debugMap;
		float o_maxDist = 15;
		float o_sigma = 8;
		float o_resolution = 0.05;
		
		cv::Mat o_edt;
		Weighting o_weighting;
		float o_coeff = 1;

};

#endif