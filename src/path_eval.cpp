#include "./../include/gauss_traj_estimator/path_eval.hpp"
#define EPS 1e-5




using namespace std;

typedef unsigned int uint;


PathEvaluator::PathEvaluator()
{

    cout << "Instance of PathEvaluator has been initialized successfully." << endl;

};

PathEvaluator::~PathEvaluator()
{
};
 

void PathEvaluator::setParams(const string mapfile, const double rs, const double grh)
{
    this->r_safe = rs;
    this->ground_rejection_height = grh;
    this->map_file = mapfile;
}


void PathEvaluator::talk() {
    cout << "Path evaluator instance is talking.." << endl;
}

void PathEvaluator::load_map(const uint map_res_scaler, const string filepath)
{    
    //ground_rejection_height = -10.0;
    //r_safe = 3.4;

    //file_name = "/home/martinbuechner/catkin_ws/src/gauss_traj_estimator/worlds/map3.bt";

    // only supports octomap so far:
    std::cout << "Provided octomap file: "<<filepath<< std::endl;
    octomap::OcTree* tree_ptr = new octomap::OcTree(filepath);

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

    cout << "edf generated" << endl;

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


    // flag 
    is_map_load = true;
    cout << "MAP_LOAD:" << is_map_load << endl;

    
}




// evaluate cost at one point (see chomp_predict code: https://github.com/icsl-Jeon/chomp_predict)
double PathEvaluator::cost_at_point(geometry_msgs::Point32 p)
{    
     
    //cout<<"[cost_at_point]: " << p.x <<","<< p.y << ","<<p.z << endl;

    octomap::point3d octomap_point(p.x,p.y,p.z);
    float distance_raw;
    octomap::point3d closestObst;

    edf_ptr->getDistanceAndClosestObstacle(octomap_point, distance_raw, closestObst);
    
    
    // compute real cost from distance value 
    if (distance_raw <=0 ) {
        return (-distance_raw + 0.5*r_safe); 
    }   
    else if ((0<distance_raw) and (distance_raw < r_safe) ) {
        //return distance_raw;
        return 1/(2*r_safe)*pow(distance_raw - r_safe,2);                
    }
    else {
        return distance_raw;
    }     
    


}

// evaluate cost at one point (see chomp_predict code: https://github.com/icsl-Jeon/chomp_predict)
PathEvaluator::eval_info PathEvaluator::cost_of_path(Eigen::MatrixXd sample_path)
{    
    
 
    //cout<<"[cost_at_point]: " << p.x <<","<< p.y << ","<<p.z << endl;

    uint sample_dim = sample_path.rows();

    double sum_path_cost = 0;
				
    geometry_msgs::Point32 sample_point;


    bool rejected = false;
    
    double prev_point_cost = 0.0;

    //  go through all points of the path
    for (uint j=0; j < sample_dim; ++j) {

        sample_point.x = sample_path(j,0);
        sample_point.y = sample_path(j,1);
        sample_point.z = 0.5;

        double point_cost;

        // distance query
        octomap::point3d octomap_point(sample_point.x,sample_point.y,sample_point.z);
        float distance_raw;
        octomap::point3d closestObst;

        edf_ptr->getDistanceAndClosestObstacle(octomap_point, distance_raw, closestObst);
    
        if ((distance_raw < 0.02) or (distance_raw == 0)) {
            if (prev_point_cost > 0) {
                point_cost = prev_point_cost + 1000000; 
            }
            else {
                point_cost = 10;
            }

            prev_point_cost = point_cost;
            rejected = true;
        }
        else {
            point_cost = 0;
            prev_point_cost = 0;
        }
        
        sum_path_cost += point_cost;

        //cout << "COST OF POINT: \n" << point_cost << endl;
    }
    
    PathEvaluator::eval_info path_evaluation;
    path_evaluation.rej = rejected;
    path_evaluation.cost = sum_path_cost/sample_dim;
    


    return path_evaluation;
    //cout << "COST OF PATH: \n" << sum_path_cost << endl;
    

}



sensor_msgs::PointCloud PathEvaluator::ComputeEDF(const int map_res_scaler, const string frame_id, const string filepath) 
{
    
    octomap::OcTree* tree_ptr = new octomap::OcTree(filepath);

    double res = tree_ptr->getResolution(); // =dx

    double x,y,z;
    
    tree_ptr->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z); 
    //cout << "[PathEvaluator::ComputeEDF]: Metric min: " << x << "," << y << "," << z << endl;
    
    min.z() = ground_rejection_height;
    
    tree_ptr->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);

    /* 
    double field_size_x = max.x() - min.x();
    double field_size_y = max.y() - min.y();

    int len_x = field_size_x * res; 
    int len_y = field_size_y * res;

    cout << "[PathEvaluator::ComputeEDF]: len_x " << len_x << endl;
    cout << "[PathEvaluator::ComputeEDF]: len_y " << len_y << endl;
    */

    // define ros msg data type
    sensor_msgs::PointCloud edf_field;


    edf_field.points.resize(pow(map_res_scaler,2)*max.x()*max.y());
    edf_field.channels.resize(1);
    edf_field.channels[0].name = "intensity";
    edf_field.channels[0].values.resize(pow(map_res_scaler,2)*max.x()*max.y());
    
    edf_field.header.frame_id = frame_id;
    edf_field.header.stamp = ros::Time::now();
    //edf_field.channels.name = "distance";

    // cout << "[PathEvaluator::ComputeEDF]: edf_field defined " << endl;
    
    

    for (int i=0; i<max.x()*map_res_scaler; ++i) {
        for(int j=0; j<max.y()*map_res_scaler; ++j) {
            
            float a = i / double(map_res_scaler);
            float b = j / double(map_res_scaler);
        
            geometry_msgs::Point32 point3d;
            //for evaluation
            point3d.x = a; 
            point3d.y = b;
            point3d.z = 0.5;
            sensor_msgs::ChannelFloat32 intensity;
            intensity.name = "intensity";
            
            //cout << "[PathEvaluator::ComputeEDF]: cost:" << PathEvaluator::cost_at_point(point3d) << endl;
            
            edf_field.channels[0].values.push_back(PathEvaluator::cost_at_point(point3d));
            
            //for visualization
            geometry_msgs::Point32 point3dvis;
            point3dvis.x = a; 
            point3dvis.y = b;
            point3dvis.z = 0.0;
            edf_field.points.push_back(point3dvis);
        
        }
        cout << map_res_scaler << endl;
    } 
    
    return edf_field;

}

void PathEvaluator::print_query_info(octomap::point3d query, octomap::OcTreeNode* node) {
        if (node != NULL) {
        cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
        }
        else 
        cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}




int PathEvaluator::checkForCollision(octomap::OcTree* tree, double x, double y, double z) {
    /* 
    OcTreeNode *n = tree.search(x,y,z);
			if (n) {
				cout << "Value: " << n->getValue() << "\n"
			}

    return 0;
     */
}
