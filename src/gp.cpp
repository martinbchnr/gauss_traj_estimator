#include "./../include/gauss_traj_estimator/gp.hpp"
#include "./../include/gauss_traj_estimator/gaussian.hpp"

typedef unsigned int uint;

using namespace std;

#define EPS 1e-5

/* 
GP::GP(function<double(double)> m, function<double(double, double)> k, double sigma) : m(m), k(k), sigma(sigma), n(0) {
}  
*/


/* 
// function to compute the covariance of data matrices: k(X,X) or k(x)
Eigen::MatrixXd compute_cov_T(Eigen::MatrixXd X) const 
{
    int N_cols = X.cols();
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(N_cols);
    Eigen::VectorXd mean_X = (1/N)*(X.transpose() * ones);
    Eigen::MatrixXd Sigma_X = (1/N)*(X.transpose() * X - mean_X * mean_X.tranpose());
	Eigen::MatrixXd I;
	I.setIdentity(N,N);
	Eigen::MatrixXd Cov_T = Sigma_X + pow(sigma_omega,2) * I;
	return Cov_T;
} */



/* 
Eigen::MatrixXd compute_kernel(Eigen::MatrixXd X, Eigen::VectorXd x_new) const 
{
    int N_cols = X.cols();
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(N_cols);
    Eigen::VectorXd mean_X = (1/N)*(X.transpose() * ones);
    Eigen::MatrixXd Sigma_X = (1/N)*(X.transpose() * X - mean_X * mean_X.tranpose());
	Eigen::MatrixXd I;
	I.setIdentity(N,N);
	Eigen::MatrixXd Cov_T = Sigma_X + pow(sigma_omega,2) * I;
	return Cov_T;
}
*/

/* 
// accepted
double kernel(Eigen::VectorXd x_n, Eigen::VectorXd x_m) {
	
	double g_SE = 1;
	double l_SE = 2;
	double squared_distance = (x_n.transpose() * x_n) + (x_m.transpose() * x_m) - (x_n.transpose() * x_m);
	double kernel = pow(g_SE, 2) * exp((-squared_distance)/(2*pow(l_SE,2)));
	return kernel;
}
 */
double scalar_kernel(double x_1, double x_2) {
	
	double g_SE = 1;
	double l_SE = 10;
	double squared_distance = pow((x_1-x_2),2);
	double kernel = pow(g_SE, 2) * exp((-squared_distance)/(2*pow(l_SE,2)));
	return kernel;
}



/* 
// accepted
Eigen::MatrixXd kernel_cov_matrix(Eigen::MatrixXd X) {

	uint D = X.cols();
	uint N = X.rows();
	// accept only NxD sized data matrices
	Eigen::MatrixXd kernel_matrix(N,N);

	for (uint i=0; i<N; i++) {
		for (int j=0; j<N; j++) {
			kernel_matrix(i,j) = kernel(X.row(i),X.row(j));
		}	
	}
	return kernel_matrix;
}
*/

/* 
Eigen::MatrixXd augmented_kernel_cov_matrix(Eigen::MatrixXd X, Eigen::VectorXd x_new) {

	// See Bishop Eq. (6.65)
	uint D = X.cols();
	uint N = X.rows();
	// accept only NxD sized data matrices
	Eigen::MatrixXd kernel_matrix(N+1,N+1);
	Eigen::VectorXd augm_vector(N);

	Eigen::MatrixXd learning_data_cov(N,N);
	learning_data_cov = kernel_cov_matrix(X);

	for (int i=0; i<N; i++) {
		augm_vector(i) = kernel(X.row(i),x_new);
	}

	Eigen::VectorXd augm_scalar(1);
	augm_scalar = kernel(x_new,x_new);

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
*/


// accepted
Eigen::MatrixXd time_kernel(Eigen::VectorXd t_a, Eigen::VectorXd t_b) {

	// See Bishop Eq. (6.65)
	uint dim_a = t_a.rows();
	uint dim_b = t_b.rows();

	// accept only NxD sized data matrices
	Eigen::MatrixXd kernel_matrix(dim_a,dim_b);

	for (uint i=0; i<dim_a; i++) {
		for (uint j=0; j<dim_b; j++) {
			kernel_matrix(i,j) = scalar_kernel(t_a(i),t_b(j));
		}	
	}
	return kernel_matrix; // dim: NxT
}





// COMPUTE EQ 11
// Function to compute mean of y* for new data input x* (time)
Eigen::MatrixXd pred_mean(Eigen::VectorXd t_train, Eigen::VectorXd t_test, Eigen::MatrixXd X_train) 
{
	uint N = t_train.rows();
	Eigen::MatrixXd k_t_test_train;
	Eigen::MatrixXd k_t_test_train_transpose;
	Eigen::MatrixXd K_t_train;
	Eigen::MatrixXd K_t_train_inv;
	Eigen::MatrixXd mean;
	Eigen::MatrixXd I;
	I.setIdentity(N,N);
	double sigma_omega = 0.01;
    
	k_t_test_train = time_kernel(t_test,t_train);
	k_t_test_train_transpose = k_t_test_train.transpose().eval();
	//cout << "pred_mean ---------------" << endl;
	//cout << k_t_test_train << endl;


	K_t_train = time_kernel(t_train, t_train) + pow(sigma_omega,2) * I;
	K_t_train_inv = K_t_train.inverse();
	//cout << K_t_train_inv << endl;
	mean = k_t_test_train * K_t_train_inv * X_train; 
	//cout << mean.col(0) << endl;
	//(TxN)(NxN)(NxD) = (TxD)
	return mean;
}

// COMPUTE EQ 12
// Function to compute the variance of t* for new data input x*
Eigen::MatrixXd pred_var(Eigen::VectorXd t_train, Eigen::VectorXd t_test, Eigen::MatrixXd X_train) 
{
	uint N = t_train.rows();
	Eigen::MatrixXd K_t_test;
	Eigen::MatrixXd K_t_test_train;
	Eigen::MatrixXd K_t_train;
	Eigen::MatrixXd I;
	I.setIdentity(N,N);
	double sigma_omega = 0.01;

	K_t_test = time_kernel(t_test,t_test);
	cout << "K_t_test ---------------" << endl;
	cout << K_t_test << endl;

	K_t_test_train = time_kernel(t_test,t_train);
	cout << "K_t_test_train ---------------" << endl;
	cout << K_t_test_train << endl;

	K_t_train = time_kernel(t_train, t_train) + pow(sigma_omega,2) * I;
	cout << "K_t_train ---------------" << endl;
	cout << K_t_train << endl;

	Eigen::MatrixXd var;
	var = K_t_test - K_t_test_train * K_t_train.inverse() * K_t_test_train.transpose().eval();
	// (TxT)-(TxN)(NxN)(NxT)
	cout << "var ---------------" << endl;
	cout << var << endl;

	return var;
}


int main()
{
	// SCALAR KERNEL SEEMS TO WORK
	double scalar_output = scalar_kernel(0.5, 3.1);
	//cout << scalar_output << endl;

	// Create train location data
	Eigen::MatrixXd X_train_x(5,1);
  	X_train_x << 	0.0,
				1.0,
				2.0,
				2.5,
				3.0;

	Eigen::MatrixXd X_train_y(5,1);
  	X_train_y << 0.0,
				1.5,
				2.0,
				1.5,
				4.5;

	// Create train time data
	Eigen::VectorXd t_train(5);
  	t_train << 	0.0, 
	  			1.5,
				4.0, 
				7.5,
				9.0;

	Eigen::VectorXd t_test;
	t_test.setLinSpaced(10,0.0,15.0);

	Eigen::MatrixXd test_output = time_kernel(t_test, t_train);
	//cout << test_output << endl;

	// Sample the prior
	Eigen::VectorXd mu_test_x = pred_mean(t_train, t_test, X_train_x);
	Eigen::VectorXd mu_test_y = pred_mean(t_train, t_test, X_train_y);
	
	Eigen::MatrixXd Sigma_test = pred_var(t_train, t_test, X_train_x);

	for(uint i=0; i < Sigma_test.rows(); i++) {
		//covariance over time
		cout << Sigma_test(i,i) << endl;
	}
	

	/* 
	for (uint i=0; i<mu_test.rows();i++) {
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

		cout << approx_mean << endl;
		//cout<< approx_sigma << endl;

	}
	 */
	
}