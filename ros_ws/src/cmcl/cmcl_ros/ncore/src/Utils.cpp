/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                            		   #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                     				   #
#                                                                              #
#  File: Utils.cpp                                                             #
# ##############################################################################
**/

#include <math.h>
#include "Utils.h"
#include <vector>
#include <algorithm>
#include <numeric>
#include <iterator>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdlib.h>



Eigen::Matrix3f Vec2Trans(Eigen::Vector3f v)
{
	float c = cos(v(2));
	float s = sin(v(2));
	Eigen::Matrix3f trans;
	trans << c, -s, v(0),	s, c, v(1),	0, 0, 1;

	return trans;
}

float CosineSimilarity(float yaw1, float yaw2)
{
	Eigen::Vector2f unitVec1 = Eigen::Vector2f(cos(yaw1), sin(yaw1));
	Eigen::Vector2f unitVec2 = Eigen::Vector2f(cos(yaw2), sin(yaw2));

	float sim = unitVec1.dot(unitVec2);

	return sim;
}

float Wrap2Pi(float angle)
{
	float wAngle = angle;
	while (wAngle < -M_PI) wAngle += 2 * M_PI;

	while (wAngle > M_PI) wAngle -= 2 * M_PI;

	return wAngle;
}

float GetYaw(float qz, float qw)
{
	float yaw = 2 * atan2(qz, qw);
	yaw = Wrap2Pi(yaw);

	return yaw;
}

std::vector<Eigen::Vector3f> Ranges2Points(const std::vector<float>& ranges, const std::vector<float>& angles)
{
	int n = ranges.size();
	std::vector<Eigen::Vector3f> homoPoints(n);

	for(int i = 0; i < n; ++i)
	{
		float r = ranges[i];
		float a = angles[i];
		Eigen::Vector3f p = Eigen::Vector3f(r * cos(a), r * sin(a), 1);
		homoPoints[i] = p;
	}

	// consider using a matrix instead of a list for later stuff
	return homoPoints;
}



std::vector<float> Downsample(const std::vector<float>& ranges, int N)
{
	int n = ranges.size() / N;
	std::vector<float> downsampled(n);

	for(int i = 0; i < n; ++i)
	{
		downsampled[i] = ranges[i * N];
	}

	return downsampled;
}

std::vector<double> Downsample(const std::vector<double>& ranges, int N)
{
	int n = ranges.size() / N;
	std::vector<double> downsampled(n);

	for(int i = 0; i < n; ++i)
	{
		downsampled[i] = ranges[i * N];
	}

	return downsampled;
}



std::vector<float> StringToVec(std::string seq)
{
	std::vector<float> vec;

	seq.erase(std::remove(seq.begin(), seq.end(), ','), seq.end());
	seq.erase(std::remove(seq.begin(), seq.end(), '['), seq.end());
	seq.erase(std::remove(seq.begin(), seq.end(), ']'), seq.end());

	size_t pos = 0;
	std::string space_delimiter = " ";
	std::vector<std::string> words;
	while ((pos = seq.find(space_delimiter)) != std::string::npos) 
	{
        words.push_back(seq.substr(0, pos));
        seq.erase(0, pos + space_delimiter.length());
    }
    words.push_back(seq.substr(0, pos));

    for(long unsigned int i = 0; i < words.size(); ++i)
    {
    	//std::cout << words[i] << std::endl;
    	vec.push_back(std::stof(words[i]));
    }

    return vec;
}


std::vector<std::string> File2Lines(std::string filePath)
{
	std::ifstream file(filePath);
	
	std::vector<std::string> fields;

	if (file.is_open()) 
	{
	    std::string line;
	    while (std::getline(file, line)) 
	    {
	       fields.push_back(line);
	    }
	    file.close();
	}

	return fields;
}


float SampleGuassian(float sigma)
{
	float sample = 0;

	for(int i = 0; i < 12; ++i)
	{
		sample += drand48() * 2 * sigma - sigma;
	}
	sample *= 0.5;

	return sample;

}

std::vector<Particle> computeDetectedParticles(const std::vector<Particle>& particles, const Eigen::Vector3f& relTrans)
{
	size_t numParticles = particles.size();

	std::vector<Particle> detectedParticles(numParticles);
	Eigen::Vector3f relTransXY(relTrans(0), relTrans(1), 1.0);

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < numParticles; ++i)
	{
			Eigen::Matrix3f particleFrame = Vec2Trans(particles[i].pose);
			Eigen::Vector3f top = particleFrame * relTransXY;
			//top(2) =  relTrans(2) + particles[i].pose(2);
			top(2) =  particles[i].pose(2);
			detectedParticles[i].pose = top;
			detectedParticles[i].weight = particles[i].weight;
	}

	return detectedParticles;
}


/*
void ComputePolarVariance(std::vector<Eigen::Vector3f>& points, std::vector<float>& weights)
{
	int pointNum = points.size();
	double dx = 0;
	double dy = 0;
	double w = 0;
	double r = 0;

	Eigen::Vector2f mu = Eigen::Vector2f(0, 0);
	Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();

	if (pointNum == 0)
	{
		mu = Eigen::Vector2f(NAN, NAN);	
		return;
	}

	for (int i = 0; i < pointNum; ++i)
	{
		r += points[i].head(2).norm() * weights[i];
		dx += points[i](0) * weights[i]; 
		dy += points[i](1) * weights[i]; 
		w +=  weights[i];
	}

	mu(0) = r / w;
	mu(1) = Wrap2Pi(atan2(dy, dx));

	if (pointNum > 1)
	{
		for (int i = 0; i < pointNum; ++i)
		{
			float diff += points[i](0) - mu(0);

			covariance(0,0) += diff * diff * weights[i];
		}

		cov(0, 0) /= w;
		double R = sqrt(dx * dx + dy * dy);
		cov(1, 1) = 1.0 - R / w;
		// https://www.ebi.ac.uk/thornton-srv/software/PROCHECK/nmr_manual/man_cv.html
	}
}*/