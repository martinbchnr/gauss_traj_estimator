#ifndef _GAUSS_TRAJ_ESTIMATOR_H
#define _GAUSS_TRAJ_ESTIMATOR_H



#include <assert.h>
#include <Madplotlib.h>
#include <math.h>
#include <stdio.h>
#include <functional>
#include <random>
#include <vector>
#include <string>
#include <numeric>
#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>


class GP
{
private:
	// mean and covariance functions
	// std::function<double(double)> m;
	// std::function<double(double, double)> k;

	// NOISE PARAMETER
	double sigma_omega = 0.75;

	// TRAINING DATA
	// Define data matrix that holds location data
	Eigen::MatrixXd X_train;
	// Define data matrix that holds the train time data
	Eigen::VectorXd t_train;
	// Define data matrix that holds the test time data
	Eigen::VectorXd t_test;

public:
	// Create a new GP with given mean and covariance function, and noise parameter
	//GP(std::function<double(double)> m, std::function<double(double, double)> k, double sigma);
	
	// Get the mean values at given points
	Eigen::MatrixXd pred_mean(Eigen::VectorXd t_train, Eigen::VectorXd t_test, Eigen::MatrixXd X_train);

	// Get the covariances at all pairs of given points
	Eigen::MatrixXd pred_var(Eigen::VectorXd t_train, Eigen::VectorXd t_test, Eigen::MatrixXd X_train);
};

#endif