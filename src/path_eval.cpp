

#include "./../include/gauss_traj_estimator/path_eval.hpp"

// function to evaluate the EDF distance field

using namespace std;
typedef unsigned int uint;



PathEvaluator::PathEvaluator()
{
    //ground_rejection_height = 0.5;
    //r_safe = 3.4;

    cout << "Instance of PathEvaluator has been initialized successfully." << endl;

};

PathEvaluator::~PathEvaluator()
{
}


void PathEvaluator::talk() {
    cout << "Path evaluator instance is talking.." << endl;
}

void PathEvaluator::load_map()
{    
    ground_rejection_height = 0.5;
    r_safe = 3.4;

    string file_name;
    file_name = "/home/martinbuechner/catkin_ws/src/gauss_traj_estimator/worlds/map3.bt";


    //if(file_name.substr(file_name.find_last_of(".")+1)=="bt") {
    std::cout << "Provided octomap file: "<<file_name<< std::endl;
    octomap::OcTree* tree_ptr = new octomap::OcTree(file_name);

    cout << "Number of nodes: " << tree_ptr->calcNumNodes() << endl;
    


    double x,y,z;
    
    tree_ptr->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z); 
    cout << "Metric min: " << x << "," << y << "," << z << endl;
    
    min.z() = ground_rejection_height;
    
    tree_ptr->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z); 
    cout << "Metric max: " << x << "," << y << "," << z << endl;

    dx = tree_ptr->getResolution();
    
    double edf_max_dist = r_safe;
    bool unknownAsOccupied = false;


    // CONSTRUCT DISTANCE MAP
    //- the first argument ist the max distance at which distance computations are clamped
    //- the second argument is the octomap
    //- arguments 3 and 4 can be used to restrict the distance map to a subarea
    //- argument 5 defines whether unknown space is treated as occupied or free
    // The constructor copies data but does not yet compute the distance map

    edf_ptr = new DynamicEDTOctomap(edf_max_dist,tree_ptr,
        min,
        max,unknownAsOccupied);
    
    edf_ptr->update(); // To actually compute the distance map, run:

    //This is how you can query the map
    octomap::point3d p(5.0,5.0,0.6);
    //As we don't know what the dimension of the loaded map are, we modify this point
    p.x() = min.x() + 0.3 * (max.x() - min.x());
    p.y() = min.y() + 0.6 * (max.y() - min.y());
    p.z() = min.z() + 0.5 * (max.z() - min.z());

    octomap::point3d closestObst;
    float distance;

    edf_ptr->getDistanceAndClosestObstacle(p, distance, closestObst);

    std::cout<<"\n\ndistance at point "<<p.x()<<","<<p.y()<<","<<p.z()<<" is "<<distance<<std::endl;
    if(distance < edf_ptr->getMaxDist())
    std::cout<<"closest obstacle to "<<p.x()<<","<<p.y()<<","<<p.z()<<" is at "<<closestObst.x()<<","<<closestObst.y()<<","<<closestObst.z()<<std::endl;



    /* 
    octomap::OcTreeNode *n = tree_ptr->search(x,y,z);
    if(n) {
    cout << "Value: " << n->getValue() << "\n";
    }
     */

    // flag 
    is_map_load = true;
    cout << "MAP_LOAD:" << is_map_load << endl;

    
}




// evaluate cost at one point (see chomp_predict code: https://github.com/icsl-Jeon/chomp_predict)
double PathEvaluator::cost_at_point(geometry_msgs::Point p){    
    
    try {
        if (!is_problem_set)
            throw 1;
        double distance_raw = 0;

        
        distance_raw = edf_ptr->getDistance(octomap::point3d(p.x,p.y,p.z));
        

        // compute real cost from distance value 
        if (distance_raw <=0 )
           return (-distance_raw + 0.5*cost_param.r_safe); 
        else if((0<distance_raw) and (distance_raw < cost_param.r_safe) ){
            return 1/(2*cost_param.r_safe)*pow(distance_raw - cost_param.r_safe,2);                
        }else        
            return 0;

    }
    catch(exception e)
    {
        std << "error in evaulating EDT value. Is edf completely loaded?" << endl;
    }

}



sensor_msgs::PointCloud2 PathEvaluator::ComputeEDF() {

    double res = tree_ptr->getResolution(); // =dx

    double field_size_x = max.x - min.x;
    double field_size_y = max.y - min.y;




}

void PathEvaluator::print_query_info(octomap::point3d query, octomap::OcTreeNode* node) {
        if (node != NULL) {
        cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
        }
        else 
        cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}


// locate the octomap-file




int PathEvaluator::checkForCollision(octomap::OcTree* tree, double x, double y, double z) {
    /* 
    OcTreeNode *n = tree.search(x,y,z);
			if (n) {
				cout << "Value: " << n->getValue() << "\n"
			}

    return 0;
     */
}
