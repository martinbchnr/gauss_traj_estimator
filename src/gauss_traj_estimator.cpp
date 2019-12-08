#include "./../include/gauss_traj_estimator/gauss_traj_estimator.hpp"
#include "./../include/gauss_traj_estimator/path_eval.hpp"



typedef unsigned int uint;

using namespace std;


GaussTrajEstimator::GaussTrajEstimator()
{
    node_name = ros::this_node::getName();
	
};

GaussTrajEstimator::~GaussTrajEstimator()
{
}

// Store the current target position
void GaussTrajEstimator::targetPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
	target_pose_rosmsg.pose.pose.position.x = msg.pose.pose.position.x;
	target_pose_rosmsg.pose.pose.position.y = msg.pose.pose.position.y;
	target_pose_rosmsg.pose.pose.position.z = msg.pose.pose.position.z;

	target_pose = RosPoseWithCovToEigenArray(target_pose_rosmsg);

	cout << "Received target pose:" << endl;
	cout << target_pose << endl;
	
}

// Receive the position training data = via-points 
void GaussTrajEstimator::trainPosesCallback(const geometry_msgs::PoseArray msg)
{
	train_poses_rosmsg = msg;
	train_poses = GaussTrajEstimator::RosPosesToEigenArray(train_poses_rosmsg);

	train_poses_x = train_poses.col(0);
	train_poses_y = train_poses.col(1);

	cout << "Received training poses:" << endl;
	cout << train_poses_x << endl;
	cout << train_poses_y << endl;

}

// Receive the time training data = assumed time when the via points are visited
void GaussTrajEstimator::trainTimesCallback(const gauss_traj_estimator::TrainTimes msg)
{
	train_times_rosmsg = msg;
	train_times = GaussTrajEstimator::RosTimesToEigenArray(train_times_rosmsg);

	cout << "Received training times:" << endl;
	cout << train_times << endl;

}



void GaussTrajEstimator::SubscribeTrainPoses() {
    if (!train_poses_topic.empty())
	{
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), train_poses_topic.c_str());
	    train_poses_subscriber = node.subscribe(train_poses_topic,1, &GaussTrajEstimator::trainPosesCallback, this);
	}
	else
	{	
		ROS_INFO("[%s]: Variable '%s' is empty", node_name.c_str(), train_poses_topic.c_str());
	}
}

void GaussTrajEstimator::SubscribeTrainTimes() {
    if (!train_times_topic.empty())
	{
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), train_times_topic.c_str());
		train_times_subscriber = node.subscribe(train_times_topic,1, &GaussTrajEstimator::trainTimesCallback, this);
	}
	else
	{	
		ROS_INFO("[%s]: Variable '%s' is empty", node_name.c_str(), train_times_topic.c_str());
	}
}

void GaussTrajEstimator::SubscribeTargetPose() {
    if (!target_pose_topic.empty())
	{
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), target_pose_topic.c_str());
		train_times_subscriber = node.subscribe(target_pose_topic,1, &GaussTrajEstimator::targetPoseCallback, this);
	}
	else
	{	
		ROS_INFO("[%s]: Variable '%s' is empty", node_name.c_str(), target_pose_topic.c_str());
	}
}


void GaussTrajEstimator::PublishPredictions()
{

	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), target_pred_path_mean_topic.c_str());
	target_pred_path_mean_pub = node.advertise<nav_msgs::Path>(target_pred_path_mean_topic, 100);


	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), target_pred_path_cov_topic.c_str());
	target_pred_path_cov_pub = node.advertise<std_msgs::Float32MultiArray>(target_pred_path_cov_topic, 100);
}

void GaussTrajEstimator::PublishTrainingData()
{

	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), evaltd_training_points_topic.c_str());
	evaltd_training_points_pub = node.advertise<visualization_msgs::MarkerArray>(evaltd_training_points_topic, 100);

}

void GaussTrajEstimator::PublishSampledData()
{

	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), sampled_pred_paths_topic.c_str());
	sampled_pred_paths_pub = node.advertise<visualization_msgs::MarkerArray>(sampled_pred_paths_topic, 100);

	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), valid_sampled_pred_paths_topic.c_str());
	valid_sampled_pred_paths_pub = node.advertise<visualization_msgs::MarkerArray>(valid_sampled_pred_paths_topic, 100);


}

void GaussTrajEstimator::PublishValidData()
{

	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), valid_pred_path_mean_topic.c_str());
	valid_pred_path_mean_pub = node.advertise<nav_msgs::Path>(valid_pred_path_mean_topic, 100);

	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), valid_pred_path_cov_pos_topic.c_str());
	valid_pred_path_cov_pos_pub = node.advertise<nav_msgs::Path>(valid_pred_path_cov_pos_topic, 100);

	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), valid_pred_path_cov_neg_topic.c_str());
	valid_pred_path_cov_neg_pub = node.advertise<nav_msgs::Path>(valid_pred_path_cov_neg_topic, 100);


}

void GaussTrajEstimator::PublishEDF()
{

	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), edf_field_topic.c_str());
	edf_field_pub = node.advertise<sensor_msgs::PointCloud>(edf_field_topic, 100);

}



Eigen::MatrixXd GaussTrajEstimator::RosPosesToEigenArray(const geometry_msgs::PoseArray pos_array)
{
	// commented out the case for using std matrices instead of Eigen (user's choice)

	int traverse_length = pos_array.poses.size();
	Eigen::MatrixXd loc_list(traverse_length,2);

	//std::vector<std::pair<double,double>> vector;

	for (int i = 0; i < pos_array.poses.size(); ++i)
	{
		loc_list(i,0) = pos_array.poses[i].position.x;
		loc_list(i,1) = pos_array.poses[i].position.y;

		//std::pair<double,double> coord = {pos_array.goal_list[i].pose.position.x, pos_array.goal_list[i].pose.position.y};
		//vector.push_back(coord);
	}
	return loc_list;
}



Eigen::MatrixXd GaussTrajEstimator::RosTimesToEigenArray(const gauss_traj_estimator::TrainTimes times_array)
{

	Eigen::MatrixXd time_vector;

	// in case when using standard msg type:
	//std_msgs::Float32MultiArray time_msg;
	//int width = times_array.layout.dim[2].size;
	//int height = times_array.layout.dim[1].size;


	for (int i = 0; i < times_array.times.size(); ++i)
	{	
		time_vector(i,0) = times_array.times[i];		
	}
	return time_vector;
}

nav_msgs::Path GaussTrajEstimator::EigenToRosPath(const Eigen::MatrixXd matrix)
{
	int matrix_size = matrix.rows();
	nav_msgs::Path pose_path;

	for (int i = 0; i < matrix_size; ++i)
	{	
		geometry_msgs::PoseStamped single_pose;
		single_pose.pose.position.x = matrix(i,0);
		single_pose.pose.position.y = matrix(i,1);;
		single_pose.pose.position.z = 0.0;
		single_pose.header.frame_id = "/world";

		pose_path.poses.push_back(single_pose);

	}

	pose_path.header.stamp = ros::Time::now();
	pose_path.header.frame_id = "/world";

	return pose_path;
}

visualization_msgs::MarkerArray GaussTrajEstimator::EigenToRosMarkerArray(const Eigen::MatrixXd matrix)
{
	int matrix_size = matrix.rows();
	visualization_msgs::MarkerArray markers;

	for (int i = 0; i < matrix_size; ++i)
	{	
		visualization_msgs::Marker m;
		m.action = 0;
		
		m.pose.position.x = matrix(i,0);
		m.pose.position.y = matrix(i,1);;
		m.pose.position.z = 0.0;
		m.pose.orientation.w = 1.0;
		m.header.frame_id = "/world";
		m.scale.x = 0.25;
		m.scale.y = 0.25;
		m.scale.z = 0.25;
		m.color.r = 1.0;
		m.color.b = 1.0;
		m.color.a = 1.0;
		m.id = markers.markers.size();
		m.type = visualization_msgs::Marker::CUBE;
		markers.markers.push_back(m);
	}

	return markers;
}

visualization_msgs::MarkerArray GaussTrajEstimator::EigenToRosSampledPathsMarkerArray(const Eigen::MatrixXd matrix, const uint sample_count)
{
	// all sampled paths are vertically concatenated in the argument matrix that is why we convert them
	// sample by sample to the marker line.

	int matrix_size = matrix.rows();
	int sample_dim = matrix_size/sample_count; // slice out 'sample-block'
	
	visualization_msgs::MarkerArray sampled_paths;

	for (int i = 0; i < sample_count; ++i)
	{	
		
		visualization_msgs::Marker one_path;
		one_path.action = 0;

		for(int j=0; j < sample_dim; ++j) {
			geometry_msgs::Point point;
			point.x = matrix(i*sample_dim + j,0);
			point.y = matrix(i*sample_dim + j,1);
			one_path.points.push_back(point);
		}
		
		one_path.header.frame_id = "/world";
		one_path.scale.x = 0.05; // only scale x is used
		one_path.color.b = 1.0;
		one_path.color.a = 1.0;
		one_path.id = i;
		one_path.type = visualization_msgs::Marker::LINE_STRIP;
		sampled_paths.markers.push_back(one_path);
		
	}

	return sampled_paths;
}


visualization_msgs::MarkerArray GaussTrajEstimator::EigenToRosSampledPathsMarkerArrayColored(const Eigen::MatrixXd matrix, const uint sample_count, const Eigen::MatrixXd intensity)
{
	// all sampled paths are vertically concatenated in the argument matrix that is why we convert them
	// sample by sample to the marker line.

	int matrix_size = matrix.rows();
	int sample_dim = matrix_size/sample_count; // slice out 'sample-block'
	
	visualization_msgs::MarkerArray sampled_paths;

	for (int i = 0; i < sample_count; ++i)
	{	
		
		visualization_msgs::Marker one_path;
		one_path.action = 0;

		for(int j=0; j < sample_dim; ++j) {
			geometry_msgs::Point point;
			point.x = matrix(i*sample_dim + j,0);
			point.y = matrix(i*sample_dim + j,1);
			one_path.points.push_back(point);
		}
		
		one_path.header.frame_id = "/world";
		one_path.scale.x = 0.05; // only scale x is used
		one_path.color.b = 0.5;
		one_path.color.r = 0.5;
		//one_path.color.a = 1.0;
		one_path.color.a = 1-intensity(i)-0.5; 
		one_path.id = i;
		one_path.type = visualization_msgs::Marker::LINE_STRIP;
		sampled_paths.markers.push_back(one_path);
		
	}

	return sampled_paths;
}






Eigen::MatrixXd GaussTrajEstimator::RosPoseWithCovToEigenArray(const geometry_msgs::PoseWithCovarianceStamped pose) 
{
	Eigen::MatrixXd eig_pose(2,1);
	
	eig_pose(0,0) = pose.pose.pose.position.x;
	eig_pose(1,0) = pose.pose.pose.position.y;

	return eig_pose;
}

std_msgs::Float32MultiArray GaussTrajEstimator::EigenToRosTimeArray(const Eigen::MatrixXd time_matrix)
{
	std_msgs::Float32MultiArray time_msg;

	for (int i = 0; i < time_matrix.rows(); ++i)
	{	
		time_msg.data.push_back(time_matrix(i));
	}
	return time_msg;
}

std_msgs::Float32MultiArray GaussTrajEstimator::EigenToRosSigmaArray(const Eigen::MatrixXd sigma_matrix)
{
	std_msgs::Float32MultiArray sigma_msg;

	sigma_msg.layout.dim[0].size = sigma_matrix.rows();
	sigma_msg.layout.dim[0].stride = sigma_matrix.rows() * sigma_matrix.cols();
	sigma_msg.layout.dim[1].size   = sigma_matrix.cols();
	sigma_msg.layout.dim[1].stride = 1*sigma_matrix.cols();
	sigma_msg.layout.dim[2].size   = 1;
	sigma_msg.layout.dim[2].stride = 1;
	
	for (int i = 0; i < sigma_matrix.rows(); ++i)
	{
		for (int j=0; j < sigma_matrix.cols(); ++j) 
		{	
			uint prob_index = 1*sigma_matrix.rows()*sigma_matrix.cols() + + i * sigma_matrix.cols() + j;
			sigma_msg.data[prob_index] = sigma_matrix(i,j);
			//multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
		}	
	}

	return sigma_msg;
}


void GaussTrajEstimator::spin() {

	bool first_run = true;

    ros::Rate r(0.4);
    while(ros::ok()) {
        ros::spinOnce();
		bool lost_track = true;
        if(lost_track) {

			// Create train location data
			Eigen::MatrixXd X_train_x(5,1);
			X_train_x << 0.0,
						1.0,
						2.0,
						2.5,
						3.0;

			Eigen::MatrixXd X_train_y(5,1);
			X_train_y << 0.0,
						1.5,
						2.0,
						1.5,
						4.5;

			Eigen::MatrixXd X_train(6,2);
			X_train  << 0.0, 0.0,
						2.0, 0.0,
						3.4578,4.1620,  
						6.7478, 8.0200,
						8.3223, 3.9896,
						11.4981, 8.0515;

			// Create train time data
			Eigen::VectorXd t_train(6);
			t_train << 	0.0,
						0.8, 
						 3.0,
						5.0,
						7.0,
						9.0;


			// epsilon follow up method:
			// use real-time velocity/orientation vector of the target  

			Eigen::VectorXd t_test;
			t_test.setLinSpaced(200,0.0,t_train(t_train.rows()-1));

			// Initialize Gaussian process with these parameters
			// {signal var, length scale, noise var}
			GP gp_debug {1.8, 2.5, 0.1}; // working: {1.8, 2.5, 0.1} default: {g=1, l=10, 0.01} 

			// Run the mean and covariance computation using the provided test and training data
			Eigen::MatrixXd mu_debug = gp_debug.pred_mean(X_train, t_train, t_test);
			Eigen::MatrixXd sigma_debug = gp_debug.pred_var(t_train, t_test);
			
			// Console output of computations to check validity
			//cout << "----- GaussTrajEstimator: debug output -----" << endl;
			//cout << mu_debug << endl;
			//cout << sigma_debug << endl;


			// Generate ROS compatible topics out of computation results
			// and publish topic data
			
			pred_path_mean_rosmsg = GaussTrajEstimator::EigenToRosPath(mu_debug);
			target_pred_path_mean_pub.publish(pred_path_mean_rosmsg);
			

			// Generate ROS messages out of the training data for visualization
			visualization_msgs::MarkerArray training_markers;
			training_markers = GaussTrajEstimator::EigenToRosMarkerArray(X_train);
			evaltd_training_points_pub.publish(training_markers);

			// Use derived mean and sigma data to sample multiple new paths
			MultiGaussian gaussian_debug(mu_debug,sigma_debug);
    		


			/*
			// Option A: Only one sample at a time: 	
			Eigen::MatrixXd single_debug_sample = gaussian_debug.sample();
			
			// Console output of sample results to check validity
			cout << "----- GaussTrajEstimator: sampled paths -----" << endl;
			cout << single_debug_sample << endl;
			*/

			
			

			
 			

			// Option B: Sample a multitude of paths at a time:

			uint valid_paths_counter = 0;
			
			 
			//PathEvaluator path_cost_eval;

			uint sample_count = 500;
			uint sample_dim = mu_debug.rows();

			Eigen::MatrixXd path_costs(sample_count, 1);

			Eigen::MatrixXd sampled_sample;
			Eigen::MatrixXd entire_sampled_data(sample_count*sample_dim, mu_debug.cols());
			
			
			Eigen::MatrixXd valid_sampled_idx(sample_count,1);

			// Go through sampled paths and evaluate cost
			for (uint i = 0; i < sample_count; i++)
			{
				sampled_sample = gaussian_debug.sample();
				entire_sampled_data.block(i*sample_dim,0,sample_dim,mu_debug.cols()) = sampled_sample; // <-- syntax is the same for vertical and horizontal concatenation
				
				PathEvaluator::eval_info result;
    			result = path_cost_eval.cost_of_path(sampled_sample);
				path_costs(i) = result.cost;

				if (result.rej == false) {
					
					valid_paths_counter += 1;
					valid_sampled_idx(i,0) = 1;
					
					//cout << "PATH NO. " << i << "IS VALID" << endl;
					
				}
				else {
					valid_sampled_idx(i,0) = 0;	
				}
				
			}

			//cout << path_costs << endl;
			cout << entire_sampled_data.rows() << endl;
			
			

			// generate matrix of valid sampled data (no collisions)
			Eigen::MatrixXd valid_sampled_data(valid_paths_counter*sample_dim,mu_debug.cols());

			uint next_row_idx = 0;
			
			for (uint k = 0; k < sample_count; k++)
			{
				
				if (valid_sampled_idx(k,0)==1) {
					valid_sampled_data.block(next_row_idx,0,sample_dim,mu_debug.cols()) = entire_sampled_data.block(k*sample_dim,0,sample_dim,mu_debug.cols());
					next_row_idx += sample_dim;
				}
				

			}

			// get location of maximum
			Eigen::MatrixXf::Index maxRow, maxCol;
  			float max_cost = path_costs.maxCoeff(&maxRow, &maxCol);

			// get location of minimum
			Eigen::MatrixXf::Index minRow, minCol;
			float min_cost = path_costs.minCoeff(&minRow, &minCol);

			
			Eigen::MatrixXd valid_mean_path_cost(sample_dim,mu_debug.cols());

			// Generate a mean-path based on cost evaluation of all paths
			for (uint k = 0; k < sample_dim; k++) {
				// per path-point
				Eigen::MatrixXd mean_per_dim(1,mu_debug.cols());

				double x_cumulated = 0.0;
				double y_cumulated = 0.0;

				double cost_cumulated = 0.0;

				//add up coordinates of all samples
				for (uint l = 0; l < sample_count; l++) {
					double cost_factor = (1-(path_costs(l)-min_cost)/(max_cost-min_cost));
					//double cost_factor = (1/(path_costs(l)));

					x_cumulated += cost_factor*entire_sampled_data(l*sample_dim+k,0);
					y_cumulated += cost_factor*entire_sampled_data(l*sample_dim+k,1);
					cost_cumulated += cost_factor;
				}

				
				mean_per_dim(0,0) = x_cumulated/cost_cumulated;
				mean_per_dim(0,1) = y_cumulated/cost_cumulated;
				
				
				valid_mean_path_cost(k,0) = mean_per_dim(0,0);
				valid_mean_path_cost(k,1) = mean_per_dim(0,1);	
			}


			cout << "VALID MEAN PATH" << endl;
			//cout << valid_mean_path_cost << endl;


			Eigen::MatrixXd valid_mean_path_rej(sample_dim,mu_debug.cols());

			// Generate a mean-path based on non-colliding paths
			for (uint k = 0; k < sample_dim; k++) {
				
				// per path-point
				Eigen::MatrixXd mean_per_dim(1,mu_debug.cols());

				double x_cumulated = 0.0;
				double y_cumulated = 0.0;

				//add up coordinates of all samples
				for (uint l = 0; l < valid_paths_counter; l++) {
					x_cumulated += valid_sampled_data(l*sample_dim+k,0);
					y_cumulated += valid_sampled_data(l*sample_dim+k,1);
				}

				if (valid_paths_counter > 0) {
					mean_per_dim(0,0) = x_cumulated/valid_paths_counter;
					mean_per_dim(0,1) = y_cumulated/valid_paths_counter;
				}
				else {
					mean_per_dim(0,0) = 0.0;
					mean_per_dim(0,1) = 0.0;
				}
				
				valid_mean_path_rej(k,0) = mean_per_dim(0,0);
				valid_mean_path_rej(k,1) = mean_per_dim(0,1);	
			}



			Eigen::MatrixXd a_new_mean_path(sample_dim,mu_debug.cols());

			// NEW AVERAGING FUNCTION
			for (int k=0; k < valid_paths_counter; k++) {
				
				for (int j=0; j < sample_dim; j++) {
					a_new_mean_path(j,0) = valid_sampled_data(k*sample_dim+j,0)/(float)valid_paths_counter;
					a_new_mean_path(j,1) = valid_sampled_data(k*sample_dim+j,1)/(float)valid_paths_counter; 
				}
			}



			for (uint k = 0; k < sample_dim; k++) {
				// per path-point
				Eigen::MatrixXd mean_per_dim(1,mu_debug.cols());

				double x_cumulated = 0.0;
				double y_cumulated = 0.0;

				//add up coordinates of all samples
				for (uint l = 0; l < valid_paths_counter; l++) {
					x_cumulated += valid_sampled_data(l*sample_dim+k,0);
					y_cumulated += valid_sampled_data(l*sample_dim+k,1);
				}

				if (valid_paths_counter > 0) {
					mean_per_dim(0,0) = x_cumulated/valid_paths_counter;
					mean_per_dim(0,1) = y_cumulated/valid_paths_counter;
				}
				else {
					mean_per_dim(0,0) = 0.0;
					mean_per_dim(0,1) = 0.0;
				}
				
				valid_mean_path_rej(k,0) = mean_per_dim(0,0);
				valid_mean_path_rej(k,1) = mean_per_dim(0,1);	
			}




			// NEW METHOD TO EVALUATE THE FINAL MEAN PATH
			
			//MultiGaussian gaussian_debug();

			Eigen::MatrixXd mean_path(sample_dim,mu_debug.cols());

			for (uint j=0; j < sample_dim; j++) {
					// loop through all dimension of a path
					Eigen::MatrixXd dim_agg(valid_paths_counter,2);

				for (uint k = 0; k < valid_paths_counter; k++) {
					// loop through all valid paths one by one
					Eigen::MatrixXd single_mean_dim(1,2);
					dim_agg(k,0) = valid_sampled_data(k*sample_dim+j,0);
					dim_agg(k,1) = valid_sampled_data(k*sample_dim+j,1);
				}
				
				// Calculate the mean and covariance of the produced sampled points
				
				
				Eigen::MatrixXd approx_mean(1,2);
				//Eigen::MatrixXd approx_sigma(2, 2);
				approx_mean.setZero();
				//approx_sigma.setZero();

				for (unsigned int i = 0; i < valid_paths_counter; i++)
				{
					approx_mean  = approx_mean  + dim_agg.row(i);
					//approx_sigma = approx_sigma + data.row(i) * data.row(i).transpose();
				}

				approx_mean  = approx_mean  / static_cast<double>(valid_paths_counter);
				//approx_sigma = approx_sigma / static_cast<double>(points);
				//approx_sigma = approx_sigma - approx_mean * approx_mean.transpose();

				cout<< approx_mean << endl;
				//cout<< approx_sigma << endl;

				mean_path(j,0) = approx_mean(0,0);
				mean_path(j,1) = approx_mean(0,1);

			}

			Eigen::MatrixXd mu_valid_approx(mu_debug.rows(),mu_debug.cols());
			Eigen::MatrixXd sigma_valid_approx(mu_debug.rows(), mu_debug.rows());

			mu_valid_approx.setZero();
			sigma_valid_approx.setZero();

			MultiGaussian valid_path_approx(mu_valid_approx, sigma_valid_approx);

			MultiGaussian::normal_params approx_mean_path_params;

			approx_mean_path_params = valid_path_approx.approximate(valid_sampled_data, valid_paths_counter);

			
			valid_mean_path_rosmsg = GaussTrajEstimator::EigenToRosPath(approx_mean_path_params.mean);
			valid_pred_path_mean_pub.publish(valid_mean_path_rosmsg);

			Eigen::MatrixXd valid_pred_path_cov_pos = approx_mean_path_params.mean;
			valid_pred_path_cov_pos.col(0) = valid_pred_path_cov_pos.col(0) + 1.96 * approx_mean_path_params.sigma.diagonal();
			valid_pred_path_cov_pos.col(1) = valid_pred_path_cov_pos.col(1) + 1.96 * approx_mean_path_params.sigma.diagonal();

			Eigen::MatrixXd valid_pred_path_cov_neg = approx_mean_path_params.mean;
			valid_pred_path_cov_neg.col(0) = valid_pred_path_cov_neg.col(0) - 1.96 * approx_mean_path_params.sigma.diagonal();
			valid_pred_path_cov_neg.col(1) = valid_pred_path_cov_neg.col(1) - 1.96 * approx_mean_path_params.sigma.diagonal();



			//Eigen::MatrixXd valid_pred_path_cov_pos = approx_mean_path_params.mean.col(0) + 1.96 * approx_mean_path_params.sigma.diagonal();
			//Eigen::MatrixXd valid_pred_path_cov_neg = approx_mean_path_params.mean - 1.96 * approx_mean_path_params.sigma.diagonal();

			valid_pred_path_cov_pos_rosmsg = GaussTrajEstimator::EigenToRosPath(valid_pred_path_cov_pos);
			valid_pred_path_cov_neg_rosmsg = GaussTrajEstimator::EigenToRosPath(valid_pred_path_cov_neg);
			valid_pred_path_cov_pos_pub.publish(valid_pred_path_cov_pos_rosmsg);
			valid_pred_path_cov_neg_pub.publish(valid_pred_path_cov_neg_rosmsg);



/* 
			// check if mean path does not collide with any walls:
			PathEvaluator::eval_info result;
			result = path_cost_eval.cost_of_path(a_new_mean_path);
			if (result.rej == true) {
				//cout << "MEAN PATH IS VALID" << endl;
				
				latest_valid_mean_path_rosmsg = valid_mean_path_rosmsg;
			}
			else {
				cout << "MEAN PATH NOT VALID, INCREASE SAMPLE NUMBER OR CHECK FOR DIVERGENCE OF PATHS." << endl;
				valid_pred_path_mean_pub.publish(latest_valid_mean_path_rosmsg);
			}
 */
			

			//cout << "LENGTH OF VALID SAMPLED DATA ARRAY:" << valid_sampled_data.rows()/sample_dim  << endl;

			
			cout << "% OF VALID PATHS:" << (float)valid_paths_counter / (float)sample_count  << endl;

			
			//cout << "COSTS OF SAMPLED PATHS: \n" << path_costs << endl;
			
			

			sampled_pred_path_rosmsg = GaussTrajEstimator::EigenToRosSampledPathsMarkerArrayColored(entire_sampled_data, sample_count, path_costs);
			sampled_pred_paths_pub.publish(sampled_pred_path_rosmsg);

			
			if(valid_paths_counter > 0 ) {
				valid_sampled_pred_path_rosmsg = GaussTrajEstimator::EigenToRosSampledPathsMarkerArray(valid_sampled_data, valid_paths_counter);
				valid_sampled_pred_paths_pub.publish(valid_sampled_pred_path_rosmsg);

				//cout << "VALID SAMPLED PATHS: \n" << valid_sampled_data << endl;
			}
			else {
				cout << "NO VALID SAMPLE PATHS, INCREASE NUMBER OF SAMPLES";
			}

        }
        r.sleep();
    }
}


void GaussTrajEstimator::GenerateEDF() {
	
	// Generate one instance of the PathEvaluator class to compute the distance field for plotting
	PathEvaluator edf_plot;
	
	// Give activity notice and compute the distance field after import of the octomap file
	edf_plot.talk();

	

	// Use some condition to only publish the discretized EDF plot
	// when a certain condition is satisfied (e.g change in the environment
	// or here: only on first execution)
			
	ros::Duration(2.0).sleep();

	edf_plot.load_map();
	// compute the discretized Euclidean distance field for plotting and publish it
	sensor_msgs::PointCloud computed_edf_field;
	computed_edf_field = edf_plot.ComputeEDF();

	edf_field_pub.publish(computed_edf_field);

	// also load the map for the path evaluator
	path_cost_eval.load_map();

}

/* void GaussTrajEstimator::InitializeGP() {
	
	// Initialize Gaussian process with these parameters
	// {signal var, length scale, noise var}
	GP gp_debug {1.8, 2.5, 0.1}; // default: {g=1, l=10, 0.01} 

}
 */



int main(int argc, char **argv)
{
    ros::init(argc, argv, "gauss_traj_estimator");
    GaussTrajEstimator gaussian_traj_estimator;

	gaussian_traj_estimator.SubscribeTrainPoses();
    gaussian_traj_estimator.SubscribeTrainTimes();
    gaussian_traj_estimator.SubscribeTargetPose();
	gaussian_traj_estimator.PublishPredictions();
	gaussian_traj_estimator.PublishTrainingData();
	gaussian_traj_estimator.PublishSampledData();
	gaussian_traj_estimator.PublishValidData();
	gaussian_traj_estimator.PublishEDF();

	
	
	cout << "Subscribers and Publishers intialized" << endl;

	// Plot Euclidean distance field
	gaussian_traj_estimator.GenerateEDF();

	

	
    gaussian_traj_estimator.spin();
    return 0;
}