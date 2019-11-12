#include "../include/gauss_traj_estimator.h"
#include "../include/gp.hpp"

GaussTrajEstimator::GaussTrajEstimator()
{
    node_name = ros::this_node::getName();
};

GaussTrajEstimator::~GaussTrajEstimator()
{
}


void GaussTrajEstimator::targetPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
	target_pose.pose.pose.position.x = msg.pose.pose.position.x;
	target_pose.pose.pose.position.y = msg.pose.pose.position.y;
	target_pose.pose.pose.position.z = msg.pose.pose.position.z;
}

void GaussTrajEstimator::trainPosesCallback(const geometry_msgs::PoseArray msg)
{
	train_poses = msg;
}

void TaskAllocator::trainTimesCallback(const geometry_msgs::PoseArray msg)
{
	train_times = msg;
}




void GaussTrajEstimator::SubscribeTrainPoses() {
    if (!train_poses.empty())
	{
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), train_poses_topic.c_str());
	    train_poses_subscriber = node.subscribe(goal_pos_topic,1, &GaussTrajEstimator::trainPosesCallback, this);
	}
	else
	{	
		ROS_INFO("[%s]: Variable '%s' is empty", node_name.c_str(), train_poses_topic.c_str());
	}
}

void GaussTrajEstimator::SubscribeTrainTimes() {
    if (!train_times.empty())
	{
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), train_times_topic.c_str());
		train_times_subscriber = node.subscribe(goal_pos_topic,1, &GaussTrajEstimator::trainTimesCallback, this);
	}
	else
	{	
		ROS_INFO("[%s]: Variable '%s' is empty", node_name.c_str(), train_times_topic.c_str());
	}
}

void GaussTrajEstimator::subscribeTargetPose() {
    if (!train_times.empty())
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
	target_pred_path_mean_pub = node.advertise<geometry_msgs::PoseArray>(pred_path_mean, 100);


	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), target_pred_path_cov_topic.c_str());
	target_pred_path_cov_pub = node.advertise<std_msgs::Float32MultiArray>(pred_path_cov, 100);
}



Eigen::MatrixXd GaussTrajEstimator::RosToEigenArray(const geometry_msgs::PoseArray pos_array)
{

	int traverse_length = pos_array.poses.size();
	Eigen::MatrixXd loc_list(traverse_length,2);

	//std::vector<std::pair<double,double>> vector;

	for (int i = 0; i < pos_array.poses.size(); ++i)
	{
		loc_list(i,0) = pos_array.poses.position.x;
		loc_list(i,1) = pos_array.poses.position.y;

		//std::pair<double,double> coord = {pos_array.goal_list[i].pose.position.x, pos_array.goal_list[i].pose.position.y};
		//vector.push_back(coord);
	}
	return MatrixXd
}

geometry_msgs::PoseArray TaskAllocator::EigenToRosArray(const Eigen::MatrixXd matrix)
{
	int matrix_length = matrix.size();
	geometry_msgs::PoseArray pose_array;

	for (int i = 0; i < size; ++i)
	{
		geometry_msgs::Pose coord;
		coord.position.x = matrix(i,0);
		coord.position.y = matrix(i,1);;
		coord.position.z = 0.0;

		pose_array.poses.push_back(coord);

	}
	return pose_array;
}



void GaussTrajEstimator::spin() {

    ros::rate r(0.1);
    while(ros::ok()) {
        ros::spinOnce();
		bool lost_track = true;
        if(lost_track) {


            // Switch to target tracking prediction mode
        }
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gauss_traj_estimator");
    GaussTrajEstimator gaussian_traj_est;
    gaussian_traj_est.SubscribeTrainPoses();
    gaussian_traj_est.SubscribeTrainTimes();
    gaussian_traj_est.SubscribeTargetPose();
    gaussian_traj_est.PublishPredictions();

	GP gp_node {1, 10, 0.01};
    gaussian_traj_node.spin();
    return 0;
}