#ifndef PATH_EVAL_H
#define PATH_EVAL_H

#include <ros/ros.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <functional>
#include <random>
#include <vector>
#include <string>
#include <iostream>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>


#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeNode.h>
#include <octomap/octomap_types.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

using namespace std;

class PathEvaluator
{

    private:
	
	DynamicEDTOctomap* edf_ptr = NULL; 
    octomap::OcTree* tree_ptr = NULL;
	// private edf-field regarding the octomap used 
	
    double ground_rejection_height;             
    double r_safe; // safe clearance (outside of r_safe, cost = 0) 
	double dx;

    string map_file;

    
	bool is_map_load = false;
    

    public:

    
    struct eval_info {
        bool rej;
        double cost;
    };


	PathEvaluator();
    ~PathEvaluator();

    void setParams(const string mapfile, const double rs, const double grh);

    //sensor_msgs::PointCloud edf_field;

    void load_map(const uint map_res_scaler, const string filepath);
    sensor_msgs::PointCloud ComputeEDF(const int map_res_scaler, const string frame_id, const string filepath);

    double cost_at_point(geometry_msgs::Point32 p);    
    
    PathEvaluator::eval_info cost_of_path(Eigen::MatrixXd sample_path);

	// Return probability values
	int checkForCollision(octomap::OcTree* tree, double x, double y, double z);
    void talk();
    void print_query_info(octomap::point3d query, octomap::OcTreeNode* node);
    
	
};

#endif