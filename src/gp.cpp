#include <functional>
#include <numeric>
#include <vector>
#include <./../include/gauss_traj_estimator/gp.hpp>
#include <gaussian.hpp>

using namespace std;
typedef unsigned int uint;

#define EPS 1e-5

GP::GP(function<double(double)> m, function<double(double, double)> k, double sigma) : m(m), k(k), sigma(sigma), n(0) {



} 


/* 
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
} */



/* 
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
 */


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
Eigen::Matrix Xd time_kernel(Eigen::VectorXd t_train, Eigen::VectorXd t_test) {

	// See Bishop Eq. (6.65)
	uint N = t_train.rows();
	uint T = t_test.rows();

	// accept only NxD sized data matrices
	Eigen::MatrixXd kernel_matrix(N,T);

	Eigen::Vector1d t_train_point(1);
	Eigen::Vector1d t_test_point(1);

	for (i=0; i<N; i++) {
		for (j=0; j<T; j++) {
			t_train_point(0) = t_train.row(i);
			t_test_point(0) = t_test.row(j);

			kernel_matrix(i,j) = kernel(t_train_point,t_test_point);
		}	
	}
	return matrix; // dim: NxT
}




// Function to compute mean of y* for new data input x* (time)
Eigen::MatrixXd pred_mean(Eigen::VectorXd t_train, Eigen::VectorXd t_test, Eigen::MatrixXd X_train) const 
{
	Eigen::MatrixXd k;
	I = Eigen::MatrixXd::Identity(N,N);
    
	K_train_test = time_kernel(t_train,t_test);
	C_N = time_kernel(t_train, t_train) + std::pow(sigma_omega,2) * I;

	Eigen::MatrixXd mean = k.tranpose() * C_N.inverse() * X_train; 
	//(TxN)(NxN)(NxD) = (TxD)
	return mean;
}


// Function to compute the variance of t* for new data input x*
Eigen::MatrixXd pred_var(Eigen::VectorXd t_train, Eigen::VectorXd t_test, Eigen::MatrixXd X_train) const 
{
	Eigen::VectorXd k;
	I = Eigen::MatrixXd::Identity(N,N);
    
	K_test = time_kernel(t_test,t_test);
	K_train_test = time_kernel(t_train,t_test);
	C_N = time_kernel(t_train,t_train) + std::pow(sigma_omega,2) * I;

	Eigen::MatrixXd var = K_test - K_train_test.tranpose() * C_N.inverse() * K_train_test;
	// (TxT)-(TxN)(NxN)(NxT)
	return var;
}


int main()
{
	// Create train location data
	Eigen::MatrixXd X_train(5,2);
  	X_train << 	0.0, 0.0,
				1.0, 1.5,
				2.0, 2.0,
				2.5, 1.5,
				3.0, 4.5;

	// Create train time data
	Eigen::VectorXd t_train(5);
  	t_train << 	0.0, 
	  			1.5,
				4.0, 
				7.5,
				9.0;

	t_test = Eigen::VectorXd::LinSpaced(30,0.0,15.0);

	// Sample the prior
	Eigen::VectorXd mu_test = gp.pred_mean(t_train, t_test, X_train);
	Eigen::MatrixXd Sigma_test = gp.pred_var(xs);

	for (int i=0, i<mu_test.rows(),i++) {
		MultiVarGaussian gp_gaussian(mu_test(i), Sigma_test(i));
		
		// Sample a number of points
    	const unsigned int points = 1000;
    	Eigen::MatrixXd x_sample(2, points);
    	Eigen::VectorXd vector(2);
    	for (unsigned i = 0; i < points; i++)
   		{
        	vector = gp_gaussian.sample();
        	x_sample(0, i) = vector(0);
        	x_sample(1, i) = vector(1);
    	}

		// Calculate the mean and convariance of the produces sampled points
		Eigen::VectorXd approx_mean(2);
		//Eigen::MatrixXd approx_sigma(2, 2);
		approx_mean.setZero();
		//approx_sigma.setZero();

		for (unsigned int i = 0; i < points; i++)
		{
			approx_mean  = approx_mean  + x_sample.col(i);
			//approx_sigma = approx_sigma + x_sample.col(i) * x_sample.col(i).transpose();
		}

		approx_mean  = approx_mean  / static_cast<double>(points);
		//approx_sigma = approx_sigma / static_cast<double>(points);
		//approx_sigma = approx_sigma - approx_mean * approx_mean.transpose();

		cout<< approx_mean << endl;
		//cout<< approx_sigma << endl;

	}
	
}