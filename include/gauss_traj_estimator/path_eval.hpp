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
	
	DynamicEDTOctomap *edf_ptr;
	// private edf-field regarding the octomap used 
	
    double ground_rejection_height;             
    double r_safe; // safe clearance (outside of r_safe, cost = 0) 
	double dx;

	bool is_map_load = false;
    

public:

	PathEvaluator();

	 // is map loaded 

	// Return probability values
	int checkForCollision(octomap::OcTree* tree, double x, double y, double z);

	// Sample the distribution
	void load_map(octomap::OcTree* octree_ptr);
};
