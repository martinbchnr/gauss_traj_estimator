#include "../include/gauss_traj_estimator.h"

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


void GaussTrajEstimator::ToEigenArray() {
}


void GaussTrajEstimator::ToMsgType() {
}



void GaussTrajEstimator::spin() {

    ros::rate r(1);
    while(ros::ok()) {
        ros::spinOnce();
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
    gaussian_traj_node.spin();
    return 0;
}