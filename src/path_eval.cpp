// function to evaluate the EDF distance field


void PathEvaluator::load_map(octomap::OcTree* octree_ptr){    
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
};

// locate the octomap-file

string file_name;
nh.param<string>("map_file_name",file_name,"/home/jbs/catkin_ws/chomp_predict/worlds/map3.vxblx");

if(file_name.substr(file_name.find_last_of(".")+1)=="bt"){
        std::cout << "Provided octomap file: "<<file_name<< std::endl;
        octomap::OcTree* tree = new octomap::OcTree(file_name);
        this->chomp_wrapper.load_map(tree);  
    }

OcTreeNode *n = tree.search(x,y,z);
			if (n) {
				cout << "Value: " << n->getValue() << "\n"
			}