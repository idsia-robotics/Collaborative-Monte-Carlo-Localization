/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: TestUtils.cpp             		                           		       #
# ##############################################################################
**/

#include "TestUtils.h"

std::vector<Particle> load_csv(std::string csv_path)
{
    std::vector<Particle> particles;

    std::ifstream file;
    file.open(csv_path);
    std::string line;
    while (std::getline(file, line))
    {
        std::vector<double> data;

        std::stringstream ss(line);
        while (ss.good()) 
        {
            std::string substr;
            std::getline(ss, substr, ',');
            data.push_back(std::stod(substr));
        }

        Particle p;
        p.pose = Eigen::Vector3f(data[0], data[1], data[2]);
        p.weight = data[3];

        particles.push_back(p);
    }

    file.close();

    return particles;
}

void dump_csv(std::vector<Particle>& particles, std::string csv_path)
{
    std::ofstream file;
    file.open(csv_path);

    for(int i = 0; i < particles.size(); ++i)
    {
    	Eigen::Vector3f p = particles[i].pose;
        file << p(0) << "," << p(1) << "," << p(2) << "," << particles[i].weight  << std::endl;
    }
   
    file.close();
}

void dump_csv(std::vector<Eigen::Vector3f>& points, std::string csv_path)
{
    std::ofstream file;
    file.open(csv_path);

    for(int i = 0; i < points.size(); ++i)
    {
    	Eigen::Vector3f p = points[i];
        file << p(0) << "," << p(1) << "," << p(2) <<  ",1.0" << std::endl;
    }
   
    file.close();
}

void dump_csv(Eigen::MatrixXd& data, std::string csv_path)
{
    std::ofstream file;
    file.open(csv_path);

    for(int i = 0; i < data.rows(); ++i)
    {
        file << data(i, 0) << "," << data(i, 1) << std::endl;
    }
   
    file.close();

}

std::vector<Particle> createSymmetricXDist(int n, int k)
{
	std::vector<Particle> particles(n * (k + 1));
	std::vector<Eigen::Vector3f> centers(k + 1);
	centers[0] = Eigen::Vector3f(0, 0, 0);

	float ang =  2 * M_PI / k;
    if (k == 2)
    {
        ang = M_PI;
    }
	float r  = 3.0;

	for(int c = 1; c < k + 1; ++c)
	{
		float t  = ang * c;
		centers[c] = Eigen::Vector3f( r * cos(t), r * sin(t), 0.0);
	}

	for(int c = 0; c < k + 1; ++c)
	{
		for (int i = 0; i < n; ++i)
		{
			Eigen::Vector3f rand = Eigen::Vector3f::Ones() - 2.0 * Eigen::Vector3f(drand48(), drand48(), drand48());
			particles[i + n * c].pose = centers[c] + 0.5 * rand;
			particles[i + n * c].weight = 1.0;
		}
	}

	return particles;
}

std::vector<Particle> createSymmetricXDistNoCenter(int n, int k)
{
    std::vector<Particle> particles(n * k);
    std::vector<Eigen::Vector3f> centers(k);

    float ang = 2 * M_PI / k;
    if (k == 2)
    {
        ang = M_PI;
    }
    float r  = 3.0;

    for(int c = 0; c < k; ++c)
    {
        float t  = ang * c;
        centers[c] = Eigen::Vector3f( r * cos(t), r * sin(t), 0.0);
    }

    for(int c = 0; c < k; ++c)
    {
        for (int i = 0; i < n; ++i)
        {
            Eigen::Vector3f rand = Eigen::Vector3f::Ones() - 2.0 * Eigen::Vector3f(drand48(), drand48(), drand48());
            particles[i + n * c].pose = centers[c] + 0.5 * rand;
            particles[i + n * c].weight = 1.0;
        }
    }

    return particles;
}

