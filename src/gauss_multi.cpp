#include <assert.h>
#include <math.h>
#include <numeric>
#include <random>
#include <vector>

#include <./../include/gauss_traj_estimator/gauss_multi.h>
#define EPS 1e-5

using namespace std;
typedef unsigned int uint;


// http://blog.sarantop.com/notes/mvn

class MultiVarGaussian
{
public:
  MultiVarGaussian(const Eigen::VectorXd& mu,
      const Eigen::MatrixXd& s);
  ~Mvn();
  double pdf(const Eigen::VectorXd& x) const;
  Eigen::VectorXd sample(unsigned int nr_iterations = 20) const;
  Eigen::VectorXd mean;
  Eigen::MatrixXd sigma;
};






MultiGaussian::MultiGaussian(default_random_engine gen, vector<double> mu, vector<vector<double>> Sigma) : gen(gen), mu(mu)
{
	// Check dimensionalities match!
	assert(mu.size() == Sigma.size());
	assert(Sigma.size() == Sigma[0].size());

	// Cholesky-decompose Sigma = LL^T
	L = vector<vector<double>>(mu.size(), vector<double>(mu.size(), 0.0));

	for (uint j=0;j<mu.size();j++)
	{
		for (uint i=j;i<mu.size();i++)
		{
			double s = 0.0;
			for (uint k=0;k<j;k++)
			{
				s += L[i][k] * L[j][k];
			}
			if (i == j) L[j][j] = sqrt(Sigma[j][j] + EPS - s);
			else { L[i][j] = (Sigma[i][j] - s) / L[j][j]; }
		}
	}
    // Now L contains the cholesky-decomposed Sigma
}

vector<double> MultiGaussian::sample()
{
	normal_distribution<double> N(0.0, 1.0);

	// Compute mu + A * z, where z ~ N(0, I)
	vector<double> z(mu.size());
	for (uint i=0;i<mu.size();i++)
	{
		z[i] = N(gen);
	}

	vector<double> ret(mu.size());
	for (uint i=0;i<mu.size();i++)
	{
		ret[i] = mu[i] + inner_product(L[i].begin(), L[i].end(), z.begin(), 0.0);
	}

	return ret;
}