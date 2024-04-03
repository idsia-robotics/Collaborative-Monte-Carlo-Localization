/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                            		   #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                     				   #
#                                                                              #
#  File: Utils.h                                                               #
# ##############################################################################
**/

#ifndef UTILS_H
#define UTILS_H


#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include "Particle.h"

std::vector<Particle> computeDetectedParticles(const std::vector<Particle>& particles, const Eigen::Vector3f& relTrans);


Eigen::Matrix3f Vec2Trans(Eigen::Vector3f v);

float CosineSimilarity(float yaw1, float yaw2);

float Wrap2Pi(float angle);

float GetYaw(float qz, float qw);

std::vector<Eigen::Vector3f> Ranges2Points(const std::vector<float>& ranges, const std::vector<float>& angles);

std::vector<float> Downsample(const std::vector<float>& ranges, int N);

std::vector<double> Downsample(const std::vector<double>& ranges, int N);

std::vector<float> StringToVec(std::string seq);

std::vector<std::string> File2Lines(std::string filePath); 

float SampleGuassian(float sigma);

#endif