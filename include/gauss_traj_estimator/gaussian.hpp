#include <random>
#include <vector>



class MultiVarGaussian
{
private:
	

	// mean vector, covariance matrix
	
    Eigen::VectorXd mean;
    Eigen::MatrixXd sigma;

public:
	// Create a new multivariate distribution N(mu, Sigma), with engine gen
	MultiVarGaussian(const Eigen::VectorXd& mu, const Eigen::MatrixXd& s);

	// Return probability values
	double pdf(const Eigen::VectorXd& x);

	// Sample the distribution
	Eigen::VectorXd sample();
};

