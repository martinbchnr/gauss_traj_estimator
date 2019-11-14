#include "./../include/gauss_traj_estimator/gauss_traj_estimator.hpp"


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
	target_pose.pose.pose.position.x = msg.pose.pose.position.x;
	target_pose.pose.pose.position.y = msg.pose.pose.position.y;
	target_pose.pose.pose.position.z = msg.pose.pose.position.z;
}

// Receive the position training data = via-points 
void GaussTrajEstimator::trainPosesCallback(const geometry_msgs::PoseArray msg)
{
	train_poses = msg;
}

// Receive the time training data = assumed time when the via points are visited
void GaussTrajEstimator::trainTimesCallback(const std_msgs::Float32MultiArray msg)
{
	train_times = msg;
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
	target_pred_path_mean_pub = node.advertise<geometry_msgs::PoseArray>(target_pred_path_mean_topic, 100);


	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), target_pred_path_cov_topic.c_str());
	target_pred_path_cov_pub = node.advertise<std_msgs::Float32MultiArray>(target_pred_path_cov_topic, 100);
}




Eigen::MatrixXd GaussTrajEstimator::RosPoseToEigenArray(const geometry_msgs::PoseArray pos_array)
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

geometry_msgs::PoseArray GaussTrajEstimator::EigenToRosPoseArray(const Eigen::MatrixXd matrix)
{
	int matrix_size = matrix.rows();
	geometry_msgs::PoseArray pose_array;

	for (int i = 0; i < matrix_size; ++i)
	{	
		geometry_msgs::Pose coord;
		coord.position.x = matrix(i,0);
		coord.position.y = matrix(i,1);;
		coord.position.z = 0.0;

		pose_array.poses.push_back(coord);

	}
	return pose_array;
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

    ros::Rate r(0.1);
    while(ros::ok()) {
        ros::spinOnce();
		bool lost_track = true;
        if(lost_track) {

			cout << "hallo" << endl;
            // Switch to target tracking prediction mode
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

	//GP gp_node {1, 10, 0.01};
    gaussian_traj_estimator.spin();
    return 0;
}