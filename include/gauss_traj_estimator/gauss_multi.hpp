#ifndef _GAUSS_MULTI
#define _GAUSS_MULTI

#include <random>
#include <vector>

/*
 A multivariate Gaussian distribution is denoted by N(mu, Sigma),
 where mu is the mean vector and Sigma is the covariance matrix.
 It may be sampled by obtaining a matrix A s.t. A * A^T = Sigma
 (here, Cholesky decomposition will be used), and then returning:
 	mu + A * z
 where z ~ N(0, I) (which is easy to sample as z_i are independent).
*/

class MultiGaussian
{
private:
	// The random engine used for sampling
	std::default_random_engine gen;

	// mean vector and decomposed covariance matrix
	std::vector<double> mu;
	std::vector<std::vector<double>> L;

public:
	// Create a new multivariate distribution N(mu, Sigma), with engine gen
	MultiGaussian(std::default_random_engine gen, std::vector<double> mu, std::vector<std::vector<double>> Sigma);

	// Sample the distribution
	std::vector<double> sample();
};

#endif