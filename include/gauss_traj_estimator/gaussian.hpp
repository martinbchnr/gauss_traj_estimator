#include <random>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>

class MultiGaussian
{
private:
	

	// mean vector, covariance matrix
    Eigen::MatrixXd mean;
    Eigen::MatrixXd sigma;


public:
	// Create a new multivariate distribution N(mu, Sigma), with engine gen
	MultiGaussian(const Eigen::MatrixXd& mu, const Eigen::MatrixXd& s);
	~MultiGaussian();

	// Return probability values
	double pdf(const Eigen::VectorXd& x) const;

	struct normal_params {
        Eigen::MatrixXd mean;
		Eigen::MatrixXd sigma;
    };


	// Sample the distribution
	Eigen::MatrixXd sample() const;
	MultiGaussian::normal_params approximate(Eigen::MatrixXd data, uint points) const;
	Eigen::MatrixXd getMean() const;
	Eigen::MatrixXd getCov() const;

};

