#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <functional>
#include <random>
#include <vector>
#include <string>

#include <gauss_multi.h>
#include <gp.h>

using namespace std;
typedef unsigned int uint;

// Random engine to use throughout
default_random_engine gen;

// How many functions to sample at each step
int n_samp = 5;

int main()
{
	// Create a new Gaussian process... 
	// use zero-mean, squared-exponential covariance (hyperparam l = 0.75), no noise.
	auto m = [](double) { return 0; };
	auto k = [](double x, double y) { return exp(-(x - y) * (x - y) / (2.0 * 0.75 * 0.75)); };
	GP gp(m, k, 0.0);

	// Condition it on two points
	gp.push(-3.0, 4.0);
	gp.push(2.0, -3.0);

	// points to be used to plot lines
	vector<double> xs;
	for (double x=-5.0;x<=5.0;x+=0.01) xs.push_back(x);

	// Sample the prior
	vector<double> mu = gp.get_means(xs);
	vector<vector<double>> Sigma = gp.get_covar(xs);

	MultiGaussian N(gen, mu, Sigma);

	vector<vector<double>> Xs(n_samp, xs);
	vector<vector<double>> Ys(n_samp);

	for (int i=0;i<n_samp;i++)
	{
		Ys[i] = N.sample();
	}

	vector<vector<double>> X(3, xs);
	vector<vector<double>> mss(3);

	for (uint i=0;i<xs.size();i++)
	{
		double mu = gp.get_means({xs[i]})[0];
		double std = sqrt(gp.get_covar({xs[i]})[0][0]);
		mss[0].push_back(mu);
		mss[1].push_back(mu - 2 * std);
		mss[2].push_back(mu + 2 * std);
	}

	to_tikz(Xs, Ys, X, mss, "test1.tex");

	// Running into numerical issues for sampling more conditioned posteriors... just plot the mean +- 2 sigma :)

	gp.push(0.0, 1.0);
	gp.push(4.0, 2.0);

	for (uint i=0;i<xs.size();i++)
	{
		double mu = gp.get_means({xs[i]})[0];
		double std = sqrt(gp.get_covar({xs[i]})[0][0]);
		mss[0][i] = mu;
		mss[1][i] = mu - 2 * std;
		mss[2][i] = mu + 2 * std;
	}

	return 0;
}