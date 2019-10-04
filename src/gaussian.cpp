#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <functional>
#include <random>
#include <vector>
#include <string>

#include <./../include/gauss_traj_estimator/gaussian.h>
#define EPS 1e-5

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Cholesky>

using namespace std;
typedef unsigned int uint;


// http://blog.sarantop.com/notes/mvn

class MultiVarGaussian
{
    private:
    public:

    // constructor and destructor
    MultiVarGaussian(const Eigen::VectorXd& mu, const Eigen::MatrixXd& s) {
        mean = mu;
        sigma = s;
    };
    
    //~MultiVarGaussian();

    // Density function
    double pdf(const Eigen::VectorXd& x) const 
    {
        double n_dim = x.rows();
        double mahalanobis_term = (x-mean).transpose() * sigma.inverse() * (x-mean);
        double sqrt2piN = std::pow(2*M_PI, n_dim/2);
        double sigma_det_sqrt = std::pow(sigma.determinant(), 1/2);

        double normal_distrib = (1/(sigma_det_sqrt * sqrt2piN)) * exp(-0.5 * mahalanobis_term);
        return normal_distrib;
    }

    // Sample from the distribution
    
    Eigen::VectorXd sample() const 
    {
        // Use Cholesky decomposition to find upper and lower triagonal matrix of sigma
        Eigen::MatrixXd L = sigma.llt().matrixL();

        // Generate random numbers acc. to white Gaussian
        //normal_distribution<double> N(0.0, 1.0);
        
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> N{0,1};

        Eigen::VectorXd z(mean.rows());
	    // Fill rows of z with numbers between 0 and 1 from z ~ N(0, I)
        for (uint k=0; k<mean.size(); k++)  
        {
		    z[k] = N(gen);
	    }

	    Eigen::VectorXd sampled_vector(mean.rows());
        for (uint i=0; i<mean.size(); i++)
        {}
        
        sampled_vector = mean + L*z;  //(L.transpose() * z).sum();
        

	    return sampled_vector;    
    }
    
};


int main() {


    // TEST PROBABILITY DENSITY FUNCTION
    Eigen::Vector2d m(0.0, 0.0);

    Eigen::Matrix2d s;
    s(0,0) = 1.0;
    s(1,0) = 0.1;
    s(1,1) = 1.0;
    s(0,1) = 0.1;

    Eigen::Vector2d x(-0.6,-0.6);

    MultiVarGaussian test_gaussian(m,s);
    double prob = test_gaussian.pdf(x);
    
    cout << prob << endl;



   // TEST SAMPLING FUNCTION

    // Define the covariance matrix and the mean
    Eigen::MatrixXd sigma_sample(2, 2);
    sigma_sample << 10, 7,
                7, 5;
    Eigen::VectorXd mean_sample(2);
    mean_sample << 2, 2;
    MultiVarGaussian test_gaussian_sample(mean_sample, sigma_sample);

    
    
 

    // Sample a number of points
    const unsigned int points = 1000;
    Eigen::MatrixXd x_sample(2, points);
    Eigen::VectorXd vector(2);
    for (unsigned i = 0; i < points; i++)
    {
        vector = test_gaussian_sample.sample();
        x_sample(0, i) = vector(0);
        x_sample(1, i) = vector(1);
    }
    // Generates a matrix of sampled points

    // Calculate the mean and convariance of the produces sampled points
    Eigen::VectorXd approx_mean(2);
    Eigen::MatrixXd approx_sigma(2, 2);
    approx_mean.setZero();
    approx_sigma.setZero();

    for (unsigned int i = 0; i < points; i++)
    {
        approx_mean  = approx_mean  + x_sample.col(i);
        approx_sigma = approx_sigma + x_sample.col(i) * x_sample.col(i).transpose();
    }

    approx_mean  = approx_mean  / static_cast<double>(points);
    approx_sigma = approx_sigma / static_cast<double>(points);
    approx_sigma = approx_sigma - approx_mean * approx_mean.transpose();

    cout<< approx_mean << endl;
    cout<< approx_sigma << endl;


    return 0;
}