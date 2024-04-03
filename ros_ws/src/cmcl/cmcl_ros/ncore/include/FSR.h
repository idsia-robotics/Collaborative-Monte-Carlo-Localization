/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: FSR.h           					                               #
# ##############################################################################
**/


#ifndef FSR_H
#define FSR_H

#include <eigen3/Eigen/Dense>
#include <vector>

class FSR 
{
	public:

		Eigen::Vector3f SampleMotion(const Eigen::Vector3f& p1, const std::vector<Eigen::Vector3f>& command, const std::vector<float>& weights, const Eigen::Vector3f& noise);

		Eigen::Vector3f Forward(Eigen::Vector3f p1, Eigen::Vector3f u);

	    Eigen::Vector3f Backward(Eigen::Vector3f p1, Eigen::Vector3f p2);


};

#endif