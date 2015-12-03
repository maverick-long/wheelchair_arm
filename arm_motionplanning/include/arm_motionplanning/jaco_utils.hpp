/*
 * utils.cpp
 *
 */

#ifndef JACO_UTILS_HPP
#define JACO_UTILS_HPP

/*** INCLUDE FILES ***/

#pragma once

#include <json/json.h>
#include <iostream>
#include <openrave/openrave.h>
#include <openrave-0.9/openrave-core.h>
#include <trajopt/problem_description.hpp>
#include <humanoids/humanoids.hpp>
#include <utils/interpolation.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <cstring>
#include <Eigen/Core>
#include <sstream>
#include <time.h>
#include <math.h> 
#include <pthread.h>


using namespace OpenRAVE;

/* General utility functions */

std::string convertMatrixtoString(Eigen::MatrixXf &traj);

void Sleep(float s);

template <typename T>
void printcoll (T const& coll)
{
    typename T::const_iterator pos;  // iterator to iterate over coll
    typename T::const_iterator end(coll.end());  // end position

    for (pos=coll.begin(); pos!=end; ++pos) {
        std::cout << *pos << ' ';
    }
    std::cout << std::endl;
}

std::vector<double> getJointValuefromTraj(trajopt::TrajArray row);

Transform getTransformfromTraj(trajopt::TrajArray row);

std::vector<double> GetMainJoint(std::vector<double> traj,std::vector<int> activejoint);

std::vector<double> GetWholeJoint(RobotBasePtr robot, const std::vector<double>& jointvalue, const std::vector<int>& jointindex);

void vector_replace(std::vector<double>& raw_vector, int start, int end, double value);

std::vector<double> TransformtoVector(Transform T);

Eigen::MatrixXf build_traj(const std::vector<double>& pose1, const std::vector<double>& pose2, int num_step);

std::vector<double> concatenate_vectors(const std::vector<double>& a, const std::vector<double>& b);

std::vector<int> vector_arange (int max);

std::string convertDoubleVectortoString(std::vector<double>& v);

Eigen::VectorXd linspace(double a, double b, int n);

int traj_is_safe(trajopt::TrajArray& traj, OpenRAVE::RobotBasePtr robot, int n = 100);


#endif // JACO_UTILS_HPP
