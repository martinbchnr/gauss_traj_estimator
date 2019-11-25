#include "./../include/gauss_traj_estimator/gauss_traj_estimator.hpp"
//#include "./../include/gauss_traj_estimator/path_eval.hpp"

/* #include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
 */
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

			Eigen::MatrixXd X_train(5,2);
			X_train  << 0.0, 0,0,
						1.0, 1.0,  
						2.0, 2.0,
						3.0, 3.0,
						4.0; 4.0;

			// Create train time data
			Eigen::VectorXd t_train(5);
			t_train << 	0.0, 
						1.5,
						4.0, 
						7.5,
						9.0;


			// epsilon follow up method:
			// use real-time velocity/orientation vector of the target  

			Eigen::VectorXd t_test;
			t_test.setLinSpaced(10,0.0,15.0);

			// Initialize Gaussian process with these parameters
			GP gp_debug {1, 10, 0.01};

			// Run the mean and covariance computation using the provided test and training data
			Eigen::MatrixXd mu_debug = gp_debug.pred_mean(X_train, t_train, t_test);
			Eigen::MatrixXd sigma_debug = gp_debug.pred_var(t_train, t_test);
			
			// Console output of computations to check validity
			cout << "----- GaussTrajEstimator: debug output -----" << endl;
			cout << mu_debug << endl;
			cout << sigma_debug << endl;


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
			uint sample_count = 100;
			uint sample_dim = mu_debug.rows();
			Eigen::MatrixXd sampled_sample;
			Eigen::MatrixXd entire_sampled_data(sample_count*sample_dim, mu_debug.cols());
			for (unsigned i = 0; i < sample_count; i++)
			{
				sampled_sample = gaussian_debug.sample();
				entire_sampled_data.block(i*sample_dim,0,sample_dim,2) = sampled_sample; // <-- syntax is the same for vertical and horizontal concatenation
			}
			
			sampled_pred_path_rosmsg = GaussTrajEstimator::EigenToRosSampledPathsMarkerArray(entire_sampled_data, sample_count);
			sampled_pred_paths_pub.publish(sampled_pred_path_rosmsg);
			
			//PathEvaluator path_evaluator();

			
			//path_evaluator.chechForCollision(path_evaluator.);
			
        }
        r.sleep();
    }
}

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

	

	
	PathEvaluator path_evaluator;
	
	path_evaluator.talk();
	path_evaluator.load_map();
 
	cout << "nachher" << endl;

    gaussian_traj_estimator.spin();
    return 0;
}