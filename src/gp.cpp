#include <functional>
#include <numeric>
#include <vector>
#include <./../include/gauss_traj_estimator/gp.hpp>

using namespace std;
typedef unsigned int uint;

#define EPS 1e-5

GP::GP(function<double(double)> m, function<double(double, double)> k, double sigma) : m(m), k(k), sigma(sigma), n(0) { } 


// Define data matrix that holds data vectors
Eigen::MatrixXd X; 

// function to compute the covariance of data matrices: k(X,X) or k(x)
Eigen::MatrixXd Sigma(Eigen::MatrixXd X) const 
{
    int N_cols = X.cols();
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(N_cols);
    Eigen::VectorXd mean_X = (1/N)*(X.transpose() * ones);
    Eigen::MatrixXd Sigma_X = (1/N)*(X.transpose() * X - mean_X * mean_X.tranpose());
}
// code for squared exponential kernel function





void GP::push(double x, double y)
{	
	// if this is the first evidence observed, construct elementary inverse matrix
	if (n == 0)
	{
		L.push_back(vector<double>(1, 1.0/(k(x, x) + sigma)));
	}
	else
	{
		/* 
	 	 Otherwise, let:
	 		a = k(x, x) + sigma
			b = [k(x, x_1) ... k(x, x_n)]^T
		*/
		double a = k(x, x) + sigma;
		vector<double> b(n);
		for (int i=0;i<n;i++)
		{
			b[i] = k(x, xt[i]);
		}

		// Now compute a' = 1/(a - b^T * L * b)
		for (int i=0;i<n;i++)
		{
			a -= b[i] * inner_product(L[i].begin(), L[i].end(), b.begin(), 0.0);
		}
		a = 1.0 / a;

		/*
		 Now:
			L_11 = a'
			L_12 = -a' * b * L
			L_21 = -L * b^T * a'
			L_22 = L + L * b^T * a' * b * L
		*/

		vector<vector<double>> Lp(n + 1, vector<double>(n + 1));

		Lp[0][0] = a;
		
		for (int i=0;i<n;i++)
		{
			vector<double> ti(n);
			for (int j=0;j<n;j++) ti[j] = L[j][i];
			Lp[0][i + 1] = -a * inner_product(b.begin(), b.end(), ti.begin(), 0.0);
		}

		for (int i=0;i<n;i++)
		{
			Lp[i + 1][0] = -a * inner_product(L[i].begin(), L[i].end(), b.begin(), 0.0);
		}

		vector<double> lhs(n);
		vector<double> rhs(n);

		for (int i=0;i<n;i++)
		{
			lhs[i] = inner_product(L[i].begin(), L[i].end(), b.begin(), 0.0);
		}
		for (int i=0;i<n;i++)
		{
			vector<double> ti(n);
			for (int j=0;j<n;j++) ti[j] = L[j][i];
			rhs[i] = inner_product(b.begin(), b.end(), ti.begin(), 0.0);
		}

		for (int i=0;i<n;i++)
		{
			for (int j=0;j<n;j++)
			{
				Lp[i + 1][j + 1] = L[i][j] + lhs[i] * a * rhs[j];
			}
		}

		L = Lp;
	}

	xt.push_back(x);
	yt.push_back(y);
	n++;
}

vector<double> GP::get_means(vector<double> xs)
{
	vector<double> ret(xs.size());

	// Compute mean(x) + k^T * L * y
	for (uint i=0;i<xs.size();i++)
	{
		ret[i] = m(xs[i]);
	}

	for (uint i=0;i<xs.size();i++)
	{
		for (int j=0;j<n;j++)
		{
			ret[i] += k(xs[i], xt[j]) * inner_product(L[j].begin(), L[j].end(), yt.begin(), 0.0);
		}
	}

	return ret;
}

vector<vector<double>> GP::get_covar(vector<double> xs)
{
	vector<vector<double>> ret(xs.size(), vector<double>(xs.size()));

	// Compute k(xs, xs) - k(xs, x) * L * k(x, xs)
	for (uint i=0;i<xs.size();i++)
	{
		for (uint j=0;j<xs.size();j++)
		{
			ret[i][j] = k(xs[i], xs[j]);
		}
	}

	// |X * Xt| |Xt * Xt| |Xt * X|
	vector<vector<double>> lhs(xs.size(), vector<double>(n, 0.0));
	vector<vector<double>> rhs(xs.size(), vector<double>(n, 0.0));
	for (uint i=0;i<xs.size();i++)
	{
		for (int j=0;j<n;j++)
		{
			lhs[i][j] = k(xs[i], xt[j]);
		}
	}

	for (int i=0;i<n;i++)
	{
		for (uint j=0;j<xs.size();j++)
		{
			rhs[j][i] = inner_product(L[i].begin(), L[i].end(), lhs[j].begin(), 0.0);
		}
	}

	for (uint i=0;i<xs.size();i++)
	{
		for (uint j=0;j<xs.size();j++)
		{
			ret[i][j] -= inner_product(lhs[i].begin(), lhs[i].end(), rhs[j].begin(), 0.0);
		}
	}

	return ret;
}