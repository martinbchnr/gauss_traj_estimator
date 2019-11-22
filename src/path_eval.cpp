#include "./../include/gauss_traj_estimator/path_eval.hpp"


// function to evaluate the EDF distance field

using namespace std;
typedef unsigned int uint;



PathEvaluator::PathEvaluator()
{
    ground_rejection_height = 0.5;
    r_safe = 3.4;

    cout << "HALLO" << endl;

    string file_name;
    file_name = "/home/martinbuechner/catkin_ws/src/gauss_traj_estimator/worlds/map3.bt";

    cout << "HAAALOOO" << endl;

    //if(file_name.substr(file_name.find_last_of(".")+1)=="bt") {
    std::cout << "Provided octomap file: "<<file_name<< std::endl;
    octomap::OcTree* tree = new octomap::OcTree(file_name);
    //PathEvaluator::load_map(tree);  
    // EDT map scale = octomap  
    
    
    double x,y,z;
    octree_ptr->getMetricMin(x,y,z);
    octomap::point3d boundary_min(x,y,z); 
    boundary_min.z() =ground_rejection_height;
    octree_ptr->getMetricMax(x,y,z);
    octomap::point3d boundary_max(x,y,z); 
    dx = octree_ptr->getResolution();
    double edf_max_dist = r_safe;
    bool unknownAsOccupied = false;

    // EDF completed
    edf_ptr = new DynamicEDTOctomap(edf_max_dist,tree,
        boundary_min,
        boundary_max,unknownAsOccupied);
    edf_ptr->update();

    
    // flag 
    is_map_load = true;
    cout << is_map_load << endl;
    }

    OcTreeNode *n = tree.search(0.4,0.5,1.3);
    if (n) {
        cout << "Value: " << n->getValue() << "\n"
    }
	
};


void PathEvaluator::load_map(octomap::OcTree* octree_ptr)
{    
    
    // EDT map scale = octomap  
    double x,y,z;
    octree_ptr->getMetricMin(x,y,z);
    octomap::point3d boundary_min(x,y,z); 
    boundary_min.z() =ground_rejection_height;
    octree_ptr->getMetricMax(x,y,z);
    octomap::point3d boundary_max(x,y,z); 
    dx = octree_ptr->getResolution();
    double edf_max_dist = r_safe;
    bool unknownAsOccupied = false;

    // EDF completed
    edf_ptr = new DynamicEDTOctomap(edf_max_dist,octree_ptr,
        boundary_min,
        boundary_max,unknownAsOccupied);
    edf_ptr->update();

    
    // flag 
    is_map_load = true;
    cout << is_map_load << endl;

}



// locate the octomap-file




int PathEvaluator::checkForCollision(octomap::OcTree* tree, double x, double y, double z) {
    
    OcTreeNode *n = tree.search(x,y,z);
			if (n) {
				cout << "Value: " << n->getValue() << "\n"
			}

    return 0;
}
