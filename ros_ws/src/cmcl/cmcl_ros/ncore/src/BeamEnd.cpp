/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: BeamEnd.cpp                                                           #
# ##############################################################################
**/

#include "BeamEnd.h"
#include "Utils.h"

#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <chrono>

BeamEnd::BeamEnd(const std::shared_ptr<GMap>& GMap, float sigma, float maxDist, Weighting weighting )
{
	o_gmap = GMap;
	o_maxDist = maxDist;
	o_sigma = sigma;   //value of 8 for map resolution 0.05, 40 for 0.01
	o_weighting = weighting;
	o_coeff = 1.0 / sqrt(2 * M_PI * o_sigma);
	o_resolution = o_gmap->Resolution();

	cv::Mat edt;
	cv::threshold(o_gmap->Map(), o_edt, 127, 255, 0);
	cv::imwrite("TH.png", o_edt);
	o_edt = 255 - o_edt;
	cv::distanceTransform(o_edt, o_edt, cv::DIST_L2, cv::DIST_MASK_3);
	//cv::threshold(o_edt, o_edt, o_maxDist, o_maxDist, 2); //Threshold Truncated
	
	o_debugMap = cv::Mat::zeros(o_edt.size(), CV_32SC1);
	cv::imwrite("edt.png", o_edt);

}

void BeamEnd::ComputeWeights(std::vector<Particle>& particles, std::shared_ptr<LidarData> data)
{
	const std::vector<Eigen::Vector3f>& scan = data->Scan();
	const std::vector<double>& scanMask = data->Mask();

	double acc_w = 0;

	// for(int m = 0; m < o_debugMaps.size(); ++m)
	// {
	// 	o_debugMaps[m] = cv::Mat::zeros(o_edts[0][0].size(), CV_32SC1);
	// }

	#pragma omp parallel for 
	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		//auto t1 = std::chrono::high_resolution_clock::now();	
		double w = 0;
		switch(o_weighting) 
		{
		    case Weighting::NAIVE : 
		    	w = naive(particles[i], scan, scanMask);
		    	break;
		    case Weighting::GEOMETRIC : 
		    	w = geometric(particles[i], scan, scanMask);
		    	break;
		  
		}
		particles[i].weight *= w;
		acc_w += w;

		//auto t2 = std::chrono::high_resolution_clock::now();
   		//auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
   		//std::cout << "BeamEnd::Timing of compute : " << ns.count() << std::endl;
	}

	//std::cout << acc_w/particles.size() << std::endl;
}



double BeamEnd::naive(const Particle& particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask)
{
	std::vector<Eigen::Vector2f> mapPoints = scan2Map(particle, scan);
	//plotScan(particle.pose, mapPoints);

	Eigen::Vector2f br = o_gmap->BottomRight();

	double weight = 1.0;
	int nonValid = 0;

	for(long unsigned int i = 0; i < mapPoints.size(); ++i)
	{
		Eigen::Vector2f mp = mapPoints[i];

		if(scanMask[i] > 0.0)
		{
			if ((mp(0) < 0) || (mp(1) < 0) || (mp(0) > br(0)) || (mp(1) > br(1)))
			{
					++nonValid;
			}
			else
			{
				float dist = o_edt.at<float>(mp(1) ,mp(0));
				double w = getLikelihood(dist);
				weight *= w;
			}
		}
	}
	//std::cout << nonValid << std::endl;

	float penalty = pow(getLikelihood(o_maxDist), nonValid);
	weight *= penalty;

	return weight;
}

static int plot = 0;

double BeamEnd::geometric(const Particle& particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask)
{	
	std::vector<Eigen::Vector2f> mapPoints = scan2Map(particle, scan);
	//plotScan(particle.pose, mapPoints);
	//plot++;

	double weight = 1.0;
	float geoW = 1.0 / scan.size();
	int debug_id = 0;

	Eigen::Vector2f br = o_gmap->BottomRight();
	int nonValid = 0;
	int valid = 0;
	double tot_dist = 0;

	for(long unsigned int i = 0; i < mapPoints.size(); ++i)
	{
		Eigen::Vector2f mp = mapPoints[i];

		if(scanMask[i] > 0.0)
		{
			if ((mp(0) < 0) || (mp(1) < 0) || (mp(0) > br(0)) || (mp(1) > br(1)))
			{
					++nonValid;
			}
			else
			{
				
				float dist = o_edt.at<float>(mp(1) ,mp(0)) * o_resolution;
				if (dist > o_maxDist) dist = o_maxDist;
				
				tot_dist += pow(dist / o_sigma, 2.0);
				++valid;
				
			}
		}
	}

	geoW = 1.0 /(valid + nonValid);
	tot_dist +=  nonValid * pow(o_maxDist / o_sigma, 2.0);
	weight = o_coeff * exp(-0.5 * tot_dist * geoW);


	//Eigen::Vector2f pp = o_gmaps[floorID][0]->World2Map(Eigen::Vector2f(particle.pose(0), particle.pose(1)));
	//o_debugMaps[debug_id].at<int>(pp(1), pp(0)) += 1.0;

	return weight;
}


float BeamEnd::getLikelihood(float distance)
{
	float l = o_coeff * exp(-0.5 * pow(distance / o_sigma, 2));
	return l;
}


/*
void BeamEnd::plotParticles(std::vector<Particle>& particles, std::string title, bool show)
{
	cv::Mat img; 
	cv::cvtColor(Gmap->Map(), img, cv::COLOR_GRAY2BGR);

	cv::namedWindow("Particles", cv::WINDOW_NORMAL);

	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		Eigen::Vector3f p = particles[i].pose;
		Eigen::Vector2f uv = Gmap->World2Map(Eigen::Vector2f(p(0), p(1)));

		cv::circle(img, cv::Point(uv(0), uv(1)), 5,  cv::Scalar(0, 0, 255), -1);
	}
	if(show)
	{
		cv::imshow("Particles", img);
		cv::waitKey(0);
	}
	cv::imwrite("/home/nickybones/Code/YouBotMCL/nmcl/" + title + ".png", img);

	cv::destroyAllWindows();
}*/


void BeamEnd::plotScan(Eigen::Vector3f laser, std::vector<Eigen::Vector2f>& zMap)
{
	cv::Mat img; 
	cv::cvtColor(o_gmap->Map(), img, cv::COLOR_GRAY2BGR);
	
	//cv::namedWindow("Scan", cv::WINDOW_NORMAL);
	Eigen::Vector2f p = o_gmap->World2Map(Eigen::Vector2f(laser(0), laser(1)));

	cv::circle(img, cv::Point(p(0), p(1)), 5,  cv::Scalar(255, 0, 0), -1);

	for(long unsigned int i = 0; i < zMap.size(); ++i)
	{
		Eigen::Vector2f p = zMap[i];
		cv::circle(img, cv::Point(p(0), p(1)), 1,  cv::Scalar(0, 0, 255), -1);
	}

	//cv::Rect myROI(475, 475, 400, 600);
	// Crop the full image to that image contained by the rectangle myROI
	// Note that this doesn't copy the data
	//cv::Mat img_(img, myROI);

	cv::imwrite("scan" + std::to_string(plot) + ".png", img);
	//cv::imshow("Scan", img_);
	//cv::waitKey(0);

	//cv::destroyAllWindows();
}

//#include <fstream>

std::vector<Eigen::Vector2f> BeamEnd::scan2Map(const Particle& particle, const std::vector<Eigen::Vector3f>& scan)
{
	Eigen::Vector3f pose = particle.pose;
	Eigen::Matrix3f trans = Vec2Trans(pose);
	std::vector<Eigen::Vector2f> mapPoints(scan.size());

	// std::fstream fout; 
	// fout.open("scan" + std::to_string(plot) + ".csv", std::ios::out | std::ios::app); 
	// std::fstream fout2; 
	// fout2.open("scanTr" + std::to_string(plot) + ".csv", std::ios::out | std::ios::app); 
	// std::fstream fout3; 
	// fout3.open("scanMp" + std::to_string(plot) + ".csv", std::ios::out | std::ios::app); 

	for(long unsigned int i = 0; i < scan.size(); ++i)
	{

		Eigen::Vector3f ts = trans * scan[i];
		Eigen::Vector2f mp = o_gmap->World2Map(Eigen::Vector2f(ts(0), ts(1)));
		mapPoints[i] = mp;

		//fout << scan[i](0) << ", " <<  scan[i](1) << ", " <<  scan[i](2) << std::endl;
		//fout2 << ts(0) << ", " <<  ts(1) << std::endl;
		//fout3 << mp(0) << ", " <<  mp(1) << std::endl;
	}
	//fout.close();
	//fout2.close();
	//fout3.close();

	

	return mapPoints;
}