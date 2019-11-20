#include <eigen3/Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

using namespace std;

typedef unsigned int uint;

class PathEvaluator
{
private:
	

	// private edf-field regarding the octomap used 
	
    

public:
	// Create a new multivariate distribution N(mu, Sigma), with engine gen
	PathEvaluator(octomap::OcTree* octree_ptr, const Eigen::MatrixXd& s);

	// Return probability values
	bool checkForCollision();

	// Sample the distribution
	void load_map() const;
};
