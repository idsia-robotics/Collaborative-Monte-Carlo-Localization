/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: TestUtils.h             		                           		       #
# ##############################################################################
**/

#ifndef TESTUTILS_H
#define TESTUTILS_H


#include <iostream>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <chrono>

#include "DistributionExchange.h"


std::vector<Particle> load_csv(std::string csv_path);

void dump_csv(std::vector<Particle>& particles, std::string csv_path);

void dump_csv(std::vector<Eigen::Vector3f>& points, std::string csv_path);

void dump_csv(Eigen::MatrixXd& data, std::string csv_path);

std::vector<Particle> createSymmetricXDist(int n, int k);

std::vector<Particle> createSymmetricXDistNoCenter(int n, int k);


#endif