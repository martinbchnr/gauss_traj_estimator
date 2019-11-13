#include <random>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>

class MultiGaussian
{
private:
	

	// mean vector, covariance matrix
	
    Eigen::VectorXd mean;
    Eigen::MatrixXd sigma;

public:
	// Create a new multivariate distribution N(mu, Sigma), with engine gen
	MultiGaussian(const Eigen::VectorXd& mu, const Eigen::MatrixXd& s);

	// Return probability values
	double pdf(const Eigen::VectorXd& x) const;

	// Sample the distribution
	Eigen::VectorXd sample() const;
};

