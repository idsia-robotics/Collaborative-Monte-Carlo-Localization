/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: MCL.h      	          				                               #
# ##############################################################################
**/

#ifndef MCL_H
#define MCL_H

#include "FSR.h"
#include "BeamEnd.h"
#include "Resampling.h"
#include "SetStatistics.h"
#include "GMap.h"
#include <memory>
#include "LidarData.h"
#include "ParticleFilter.h"
#include "DistributionExchange.h"

class MCL
{
	public:


		//! A constructor
	    /*!
	     \param fm is a ptr to a GMap object
	      \param mm is a ptr to a MotionModel object, which is an abstract class. FSR is the implementation 
	      \param sm is a ptr to a BeamEnd object, which is an abstract class. BeamEnd is the implementation 
	      \param rs is a ptr to a Resampling object, which is an abstract class. LowVarianceResampling is the implementation 
	      \param n_particles is an int, and it defines how many particles the particle filter will use
	    */
		MCL(std::shared_ptr<GMap> gmap, std::shared_ptr<FSR> mm, std::shared_ptr<BeamEnd> sm, 
			std::shared_ptr<Resampling> rs, int n_particles);


		//! A constructor
	    /*!
	     \param fm is a ptr to a GMap object
	    */
		MCL(const std::string& jsonConfigPath);


		//! A constructor
	    /*!
	     * \param fm is a ptr to a GMap object
	      \param mm is a ptr to a MotionModel object, which is an abstract class. FSR is the implementation 
	      \param sm is a ptr to a BeamEnd object, which is an abstract class. BeamEnd is the implementation 
	      \param rs is a ptr to a Resampling object, which is an abstract class. LowVarianceResampling is the implementation 
	      \param n_particles is an int, and it defines how many particles the particle filter will use
	      \param initGuess is a vector of initial guess for the location of the robots
	      \param covariances is a vector of covariances (uncertainties) corresponding to the initial guesses
	      \param injectionRatio is an float, and it determines which portions of the particles are replaced when relocalizing
	    */
		MCL(std::shared_ptr<GMap> gmap, std::shared_ptr<FSR> mm, std::shared_ptr<BeamEnd> sm, 
			std::shared_ptr<Resampling> rs, int n_particles, std::vector<Eigen::Vector3f> initGuess, 
			std::vector<Eigen::Matrix3d> covariances);


		//! A getter for the mean and covariance of the particles
		/*!
		   \return an object SetStatistics, that has fields for mean and covariance
		*/
		SetStatistics Stats()
		{
			return o_stats;
		}


		//! A getter particles representing the pose hypotheses 
		/*!
		   \return A vector of points, where each is Eigen::Vector3f = (x, y, theta)
		*/
		std::vector<Particle> Particles()
		{
			return o_particles;
		}


		//! Advanced all particles according to the control and noise, using the chosen MotionModel's forward function
		/*!
		  \param control is a 3d control command. In the FSR model it's (forward, sideways, rotation)
		  \param odomWeights is the corresponding weight to each odometry source
	      \param noise is the corresponding noise to each control component
		*/
		void Predict(const std::vector<Eigen::Vector3f>& control, const std::vector<float>& odomWeights, const Eigen::Vector3f& noise);

		//! Considers the beamend likelihood of observation for all hypotheses, and then performs resampling 
		/*!
		  \param scan is a vector of homogeneous points (x, y, 1), in the sensor's frame. So the sensor location is (0, 0, 0)
		  		Notice that for the LaserScan messages you need to first transform the ranges to homo points, and then center them 
		*/
		void Correct(std::shared_ptr<LidarData> data);


		void FuseDistribution(std::shared_ptr<DistributionData>& distData);
		

		std::shared_ptr<DistributionData> CreateDistribution(DISTRIBUTION distType, const Eigen::Vector3f& relTrans);



		//! Initializes filter with new particles upon localization failure
		void Recover();

		Eigen::Vector3f Backward(Eigen::Vector3f p1, Eigen::Vector3f p2)
		{
			return o_motionModel->Backward(p1, p2);
		}

		//! A setter for the injection ration for particle, in case of text spotting. The ratio represents the fraction of particles that will be removed, and the same number
		// of particles will be injected at the location of detected text. The removed particles are always the ones with the lowest weights. 
		

		const cv::Mat& DebugMap()
		{
			return o_beamEndModel->DebugMap();
		}


		std::shared_ptr<GMap> Map()
		{
			return o_gmap;
		}

	private:

		void dumpParticles();
		int traceInMap(Eigen::Vector3f p1, Eigen::Vector3f p2);
		void detectionReweight(Eigen::Vector3f trans);
		//std::vector<Particle> computeDetectedParticles(const Eigen::Vector3f& relTrans);
	
		std::shared_ptr<ParticleFilter> o_particleFilter;
		unsigned int o_frame = 0;
		std::shared_ptr<FSR> o_motionModel;
		std::shared_ptr<BeamEnd> o_beamEndModel;
		std::shared_ptr<Resampling> o_resampler; 
		std::shared_ptr<DistributionExchange> o_distExcange;
		std::shared_ptr<GMap> o_gmap;
		int o_numParticles = 0;
		std::vector<Particle> o_particles;
		SetStatistics o_stats;
		Eigen::Vector2f o_detectionNoise;
};

#endif
