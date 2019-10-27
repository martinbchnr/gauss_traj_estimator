#ifndef GP_H
#define GP_H


#include <assert.h>
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
	public:
	// mean and covariance functions
	// std::function<double(double)> m;
	// std::function<double(double, double)> k;

	// NOISE PARAMETER
	double sigma_omega; // 0.01 works

	// TRAINING DATA
	// Define data matrix that holds location data
	Eigen::MatrixXd X_train;
	// Define data matrix that holds the train time data
	Eigen::VectorXd t_train;
	// Define data matrix that holds the test time data
	Eigen::VectorXd t_test;

	Eigen::MatrixXd pred_path_var;
	Eigen::MatrixXd pred_path_mean;

	
	// Compute the convetional data covariance matrix 
	Eigen::MatrixXd compute_data_cov(Eigen::MatrixXd X);
	
	// Compute the scalar kernel value based on two vectors
	double scalar_kernel_f2vect(Eigen::VectorXd x_a, Eigen::VectorXd x_b, double g, double l);

	// Compute the scalar kernel value based on two scalars
	double scalar_kernel_f2scalar(double a_1, double a_2, double g, double l);
		
	// compute the kernel-covariance matrix of a matrix X
	Eigen::MatrixXd kernel_cov_matrix_f1matrix(Eigen::MatrixXd X, double g, double l);

	// compute an augmented kernel covariance matrix based on a matrix X and an additional vector x_new
	Eigen::MatrixXd augmented_kernel_cov_matrix(Eigen::MatrixXd X, Eigen::VectorXd x_new, double g, double l);

	// Compute the kernel matrix based on two vectors
	Eigen::MatrixXd kernel_matrix_f2vect(Eigen::VectorXd x_a, Eigen::VectorXd x_b, double g, double l);
	
	// Get the mean values at given points
	Eigen::MatrixXd pred_mean(Eigen::VectorXd t_train, Eigen::VectorXd t_test, Eigen::MatrixXd X_train, double data_noise, double g, double l) 

	// Get the covariances at all pairs of given points
	Eigen::MatrixXd pred_var(Eigen::VectorXd t_train, Eigen::VectorXd t_test, Eigen::MatrixXd X_train, double data_noise, double g, double l);


};

#endif // GP_H