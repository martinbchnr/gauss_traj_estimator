#include "./../include/gauss_traj_estimator/gp.hpp"
#include "./../include/gauss_traj_estimator/gaussian.hpp"

typedef unsigned int uint;

using namespace std;

#define EPS 1e-5

// initialize 
GP::GP(double kernel_g_SE=1, double kernel_l_SE=10, double sigma_noise=0.01): g(kernel_g_SE), l(kernel_l_SE), data_noise(sigma_noise)  
{   
}

GP::~GP()
{   
}



// function to compute the covariance of data matrices: k(X,X) or k(x)
Eigen::MatrixXd GP::compute_data_cov(Eigen::MatrixXd X) 
{
    int N_cols = X.cols();
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(N_cols);
    
	Eigen::VectorXd mean_X = (1/N_cols)*(X.transpose() * ones);
    Eigen::MatrixXd Sigma_X = (1/N_cols)*(X.transpose() * X - mean_X * mean_X.transpose());
	
	return Sigma_X;
}


// compute vector-input kernel
double GP::scalar_kernel_f2vect(Eigen::VectorXd x_a, Eigen::VectorXd x_b, double g, double l) {
	
	double g_SE = g;
	double l_SE = l;
	double squared_distance = (x_a.transpose() * x_a).sum() + (x_b.transpose() * x_b).sum() - (x_a.transpose() * x_b).sum();
	double kernel = pow(g_SE, 2) * exp((-squared_distance)/(2*pow(l_SE,2)));
	return kernel;
}


// compute scalar-input kernel
double GP::scalar_kernel_f2scalar(double a_1, double a_2, double g, double l) {
	
	double g_SE = g; // =1 works
	double l_SE = l; // =10 works
	double squared_distance = pow((a_1-a_2),2);
	double kernel = pow(g_SE, 2) * exp((-squared_distance)/(2*pow(l_SE,2)));
	return kernel;
}


// compute matrix-input kernel covariance matrix
Eigen::MatrixXd GP::kernel_cov_matrix_f1matrix(Eigen::MatrixXd X, double g, double l) {

	uint D = X.cols();
	uint N = X.rows();
	// accept only NxD sized data matrices
	Eigen::MatrixXd kernel_matrix(N,N);

	for (uint i=0; i<N; i++) {
		for (int j=0; j<N; j++) {
			kernel_matrix(i,j) = GP::scalar_kernel_f2vect(X.row(i),X.row(j),g,l);
		}	
	}
	return kernel_matrix;
}



Eigen::MatrixXd GP::augmented_kernel_cov_matrix(Eigen::MatrixXd X, Eigen::VectorXd x_new, double g, double l) {

	// See Bishop Eq. (6.65)
	uint D = X.cols();
	uint N = X.rows();
	// accept only NxD sized data matrices
	Eigen::MatrixXd kernel_matrix(N+1,N+1);
	Eigen::VectorXd augm_vector(N);

	Eigen::MatrixXd learning_data_cov(N,N);
	learning_data_cov = GP::kernel_cov_matrix_f1matrix(X,g,l);

	for (int i=0; i<N; i++) {
		augm_vector(i) = GP::scalar_kernel_f2vect(X.row(i),x_new, g, l);
	}

	Eigen::VectorXd augm_scalar(1);
	augm_scalar(0) = GP::scalar_kernel_f2vect(x_new,x_new, g, l);

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




// compute kernel matrix
Eigen::MatrixXd GP::kernel_matrix_f2vect(Eigen::VectorXd x_a, Eigen::VectorXd x_b, double g, double l) {

	// See Bishop Eq. (6.65)
	uint dim_a = x_a.rows();
	uint dim_b = x_b.rows();

	// accept only NxD sized data matrices
	Eigen::MatrixXd kernel_matrix(dim_a,dim_b);

	for (uint i=0; i<dim_a; i++) {
		for (uint j=0; j<dim_b; j++) {
			kernel_matrix(i,j) = scalar_kernel_f2scalar(x_a(i),x_b(j),g,l);
		}	
	}
	return kernel_matrix; // dim: (dim_a)x(dim_b)
}





// COMPUTE EQ 11
// compute predicted mean
Eigen::MatrixXd GP::pred_mean(Eigen::MatrixXd X_train, Eigen::VectorXd t_train, Eigen::VectorXd t_test) 
{
	uint N = t_train.rows();

	Eigen::MatrixXd I;
	I.setIdentity(N,N);
	
	// noise=0.01 works
	double sigma_omega = data_noise; 

	Eigen::MatrixXd k_t_test_train = kernel_matrix_f2vect(t_test,t_train,g,l);
	Eigen::MatrixXd k_t_test_train_transpose = k_t_test_train.transpose().eval();
	// print out k_t_test_train
	cout << "pred_mean(): K_t_test_train ---------------" << endl;
	cout << k_t_test_train << endl;
	
	Eigen::MatrixXd K_t_train = kernel_matrix_f2vect(t_train, t_train,g,l) + pow(sigma_omega,2) * I;
	Eigen::MatrixXd K_t_train_inv = K_t_train.inverse();
	// print out k_t_train_inv
	cout << "pred_mean(): K_t_train_inv ---------------" << endl;
	cout << K_t_train_inv << endl;

	Eigen::MatrixXd mean;
	mean = k_t_test_train * K_t_train_inv * X_train; 

	//cout << mean.col(0) << endl;
	//(TxN)(NxN)(NxD) = (TxD)
	return mean;
}


// COMPUTE EQ 12
// Function to compute the variance of t* for new data input x*
Eigen::MatrixXd GP::pred_var(Eigen::MatrixXd X_train, Eigen::VectorXd t_train, Eigen::VectorXd t_test) 
{
	uint N = t_train.rows();

	Eigen::MatrixXd I;
	I.setIdentity(N,N);
	// noise=0.01 works
	double sigma_omega = data_noise;

	Eigen::MatrixXd K_t_test = kernel_matrix_f2vect(t_test,t_test, g, l);
	// print out K_t_test
	cout << "pred_var(): K_t_test ---------------" << endl;
	cout << K_t_test << endl;
	
	Eigen::MatrixXd K_t_test_train = kernel_matrix_f2vect(t_test,t_train, g, l);
	// print out K_t_test_train
	cout << "pred_var(): K_t_test_train ---------------" << endl;
	cout << K_t_test_train << endl;

	Eigen::MatrixXd K_t_train  = kernel_matrix_f2vect(t_train, t_train, g, l) + pow(sigma_omega,2) * I;
	// print out K_t_train
	cout << "pred_var(): K_t_train ---------------" << endl;
	cout << K_t_train << endl;
	
	Eigen::MatrixXd var;
	var = K_t_test - K_t_test_train * K_t_train.inverse() * K_t_test_train.transpose().eval();
	// print out predicted variance
	cout << "var ---------------" << endl;
	cout << var << endl;

	return var; // (TxT)-(TxN)(NxN)(NxT)
}


int main()
{

	// Create train location data
	Eigen::MatrixXd X_train_x(5,1);
  	X_train_x << 0.0,
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

	Eigen::MatrixXd X_train(5,2);
	X_train  << 0.0, 0,0,
				1.0, 1.5,  
				2.0, 2.0,
				2.5, 1.5,
				3.0; 4.5;

	// Create train time data
	Eigen::VectorXd t_train(5);
  	t_train << 	0.0, 
	  			1.5,
				4.0, 
				7.5,
				9.0;

	Eigen::VectorXd t_test;
	t_test.setLinSpaced(10,0.0,15.0);

	
	GP gp_debug {1, 10, 0.01};

	Eigen::VectorXd mu_debug = gp_debug.pred_mean(X_train, t_train, t_test);
	Eigen::VectorXd sigma_debug = gp_debug.pred_var(X_train, t_train, t_test);

	Eigen::MatrixXd test_output = gp_debug.kernel_matrix_f2vect(t_test, t_train, 1, 10);
	//cout << test_output << endl;

	// SCALAR KERNEL SEEMS TO WORK
	double scalar_output = gp_debug.scalar_kernel_f2scalar(0.5, 3.1,1,10);
	//cout << scalar_output << endl;

	// Old function call before using GP-class
	//Eigen::VectorXd mu_test_x = pred_mean(t_train, t_test, X_train_x);
	//Eigen::VectorXd mu_test_y = pred_mean(t_train, t_test, X_train_y);
	
	//Eigen::MatrixXd Sigma_test = pred_var(t_train, t_test, X_train_x);

	
	
	for(uint i=0; i < sigma_debug.rows(); i++) {
		//covariance over time
		cout << sigma_debug(i,i) << endl;
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