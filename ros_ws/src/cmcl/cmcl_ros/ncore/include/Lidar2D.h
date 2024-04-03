/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                            		   #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                     				   #
#                                                                              #
#  File: Lidar2D.h                                                             #
# ##############################################################################
**/

#ifndef LIDAR2D_H
#define LIDAR2D_H

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>


class Lidar2D
{
	public:

		Lidar2D(std::string name_, Eigen::Vector3f origin, int numBeams, float maxAngle, float minAngle);

		Lidar2D(std::string name_, std::string yamlFolder);

		Lidar2D(std::string jsonPath);

		//const std::vector<float>& Heading() const
		std::vector<float> Heading() 
		{
			return o_heading;
		}

		std::string Name()
		{
			return o_name;
		}

		Eigen::Matrix3f Trans()
		{
			return o_trans;
		}



		std::vector<Eigen::Vector3f> Center(const std::vector<float>& ranges);



	private:

		std::vector<float> o_heading;
		std::string o_name;
		Eigen::Matrix3f o_trans;

};





#endif