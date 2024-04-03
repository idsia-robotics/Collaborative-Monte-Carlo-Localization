/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                            		   #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                     				   #
#                                                                              #
#  File: Lidar2D.cpp                                                           #
# ##############################################################################
**/


#include "Lidar2D.h"
#include "Utils.h"
#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>

Lidar2D::Lidar2D(std::string name, Eigen::Vector3f origin, int numBeams, float maxAngle, float minAngle)
{
	o_name = name;
	o_trans = Vec2Trans(origin);
	

	float reso = (maxAngle - minAngle) / (numBeams - 1);
	for(int i = 0; i < numBeams; ++i)
	{
		float angle =  minAngle + i * reso;
		o_heading.push_back(angle);
	}
}

Lidar2D::Lidar2D(std::string jsonPath)
{
	using json = nlohmann::json;

	std::ifstream file(jsonPath);
	json config;
	file >> config;
	o_name = config["name"];
	float maxAngle = config["angle_max"];
	float minAngle = config["angle_min"];
	int numBeams = config["num_beams"];
	std::vector<float> origin = config["origin"];
	o_trans = Vec2Trans(Eigen::Vector3f(origin[0], origin[1], origin[2]));
	float reso = (maxAngle - minAngle) / (numBeams - 1);
	for(int i = 0; i < numBeams; ++i)
	{
		float angle =  minAngle + i * reso;
		o_heading.push_back(angle);
	}
	
}


Lidar2D::Lidar2D(std::string name, std::string yamlFolder)
{
	o_name = name;
	std::vector<std::string> fields = File2Lines(yamlFolder + name + ".yaml");

	// "angle_max: " - 11
	fields[1].erase(0,11);
	// "angle_min: " - 11
	fields[2].erase(0,11);
	// "num_beams: " - 11
	fields[3].erase(0,11);
	// "origin: " - 8
	fields[4].erase(0,8);

	float maxAngle = std::stof(fields[1]);
	float minAngle = std::stof(fields[2]);
	int numBeams = std::stoi(fields[3]);
	std::vector<float> vec = StringToVec(fields[4]);
	Eigen::Vector3f origin = Eigen::Vector3f(vec[0], vec[1], vec[2]);
	o_trans = Vec2Trans(origin);

	float reso = (maxAngle - minAngle) / (numBeams - 1);
	for(int i = 0; i < numBeams; ++i)
	{
		float angle =  minAngle + i * reso;
		o_heading.push_back(angle);
	}
}



std::vector<Eigen::Vector3f> Lidar2D::Center(const std::vector<float>& ranges)
{
	std::vector<Eigen::Vector3f> points_3d = Ranges2Points(ranges, o_heading);

	int n = points_3d.size(); 
	std::vector<Eigen::Vector3f> transPoints(n);

	for(int i = 0; i < n; i++)
	{
		Eigen::Vector3f p = points_3d[i];
		Eigen::Vector3f p_trans = o_trans * p;
		transPoints[i] = p_trans;
	}

	return transPoints;
}

