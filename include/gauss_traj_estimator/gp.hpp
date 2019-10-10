#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <functional>
#include <random>
#include <vector>
#include <string>
#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Cholesky>


class GP
{
private:
	// mean and covariance functions
	std::function<double(double)> m;
	std::function<double(double, double)> k;

	// NOISE PARAMETER
	double sigma_omega;

	// TRAINING DATA
	// Define data matrix that holds position vectors
	Eigen::MatrixXd X;
	// Define data matrix that holds the times 
	Eigen::VectorXd T:

	// current inverse gram matrix
	std::vector<std::vector<double>> L;

public:
	// Create a new GP with given mean and covariance function, and noise parameter
	GP(std::function<double(double)> m, std::function<double(double, double)> k, double sigma);

	// Get the mean values at given points
	std::vector<double> get_means(std::vector<double> xs);

	// Get the covariances at all pairs of given points
	std::vector<std::vector<double>> get_covar(std::vector<double> xs);
};

#endif