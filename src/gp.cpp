#include <functional>
#include <numeric>
#include <vector>
#include <./../include/gauss_traj_estimator/gp.hpp>

using namespace std;
typedef unsigned int uint;

#define EPS 1e-5

GP::GP(function<double(double)> m, function<double(double, double)> k, double sigma) : m(m), k(k), sigma(sigma), n(0) { } 



// function to compute the covariance of data matrices: k(X,X) or k(x)
Eigen::MatrixXd compute_cov_T(Eigen::MatrixXd X) const 
{
    int N_cols = X.cols();
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(N_cols);
    Eigen::VectorXd mean_X = (1/N)*(X.transpose() * ones);
    Eigen::MatrixXd Sigma_X = (1/N)*(X.transpose() * X - mean_X * mean_X.tranpose());
	I = Eigen::MatrixXd::Identity();
	Eigen::MatrixXd Cov_T = Sigma_X + std::pow(sigma_omega,2) * I;
	return Cov_T;
}


	

Eigen::MatrixXd compute_kernel(Eigen::MatrixXd X, Eigen::VectorXd x_new) const 
{
    int N_cols = X.cols();
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(N_cols);
    Eigen::VectorXd mean_X = (1/N)*(X.transpose() * ones);
    Eigen::MatrixXd Sigma_X = (1/N)*(X.transpose() * X - mean_X * mean_X.tranpose());
	I = Eigen::MatrixXd::Identity();
	Eigen::MatrixXd Cov_T = Sigma_X + std::pow(sigma_omega,2) * I;
	return Cov_T;
}






// accepted
std::double kernel(Eigen::VectorXd x_n, Eigen::VectorXd x_m) {
	
	std::double g_SE = 1;
	std::double l_SE = 2;
	std::double squared_distance = (x_n.tranpose() * x_n) + (x_m.tranpose() * x_m) - (x_n.tranpose() * x_m);
	std::double kernel = std::pow(g_SE, 2) * std::exp((-squared_distance)/(2*std::pow(l_SE,2)));
	return kernel;
}


// accepted
Eigen::Matrix Xd kernel_cov_matrix(Eigen::MatrixXd X) {

	uint D = X.cols();
	uint N = X.rows();
	// accept only NxD sized data matrices
	Eigen::MatrixXd kernel_matrix(N,N);

	for (i=0; i<N; i++) {
		for (j=0; j<N; j++) {
			kernel_matrix(i,j) = kernel(X.row(i),X.row(j));
		}	
	}
	return kernel_matrix;
}


// accepted
Eigen::Matrix Xd augmented_kernel_cov_matrix(Eigen::MatrixXd X, Eigen::VectorXd x_new) {

	// See Bishop Eq. (6.65)
	uint D = X.cols();
	uint N = X.rows();
	// accept only NxD sized data matrices
	Eigen::MatrixXd kernel_matrix(N+1,N+1);
	Eigen::VectorXd augm_vector(N);

	Eigen::Matrix Xd learning_data_cov(N,N);
	learning_data_cov = kernel_cov_matrix(X);

	for (i=0; i<N; i++) {
		augm_vector(i) = kernel(X.row(i),x_new);
	}

	Eigen::VectorXd augm_scalar(1);
	augm_scalar = kernel(X_new,x_new);

	// concatenate all three computed kernels/kernel matrices
	Eigen::MatrixXd upper_half(learning_data_cov.rows(), learning_data_cov.cols()+augm_vector.cols());
	upper_half << learning_data_cov, augm_vector;
	
	Eigen::MatrixXd lower_half(augm_vector.cols(), learning_data_cov.cols()+1);
	lower_half << augm_vector.transpose(), augm_scalar;

	Eigen::MatrixXd full_kernel_matrix(upper_half.rows()+lower_half.rows(), upper_half.cols());
	full_kernel_matrix << upper_half,
						  lower_half;

	return full_kernel_matrix;
}

// accepted
Eigen::Matrix Xd new_kernel_cov(Eigen::MatrixXd X, Eigen::VectorXd x_new) {

	// See Bishop Eq. (6.65)
	uint N = X.rows();
	// accept only NxD sized data matrices
	Eigen::VectorXd kernel_vector(N);

	for (i=0; i<N; i++) {
		kernel_vector(i) = kernel(X.row(i),x_new);
	}
	return kernel_vector;
}



// Function to compute mean of t* for new data input x*
std::double pred_mean(Eigen::VectorXd T, Eigen::VectorXd x_new, Eigen::MatrixXd X) const 
{
	Eigen::VectorXd k;
	I = Eigen::MatrixXd::Identity(N,N);
    
	k = new_kernel_cov(X,x_new);
	C_N = kernel_cov_matrix(X) + std::pow(sigma_omega,2) * I;

	std::double mean_for_x_new = k.tranpose() * C_N.inverse() * T;
	return mean_for_x_new;
}


// Function to compute the variance of t* for new data input x*
std::double pred_var(Eigen::VectorXd T, Eigen::VectorXd x_new, Eigen::MatrixXd X) const 
{
	Eigen::VectorXd k;
	I = Eigen::MatrixXd::Identity(N,N);
    
	k = new_kernel_cov(X,x_new);
	C_N = kernel_cov_matrix(X) + std::pow(sigma_omega,2) * I;

	std::double var_for_x_new = kernel(X_new,x_new) - k.tranpose() * C_N.inverse() * k;
	return var_for_x_new;
}


