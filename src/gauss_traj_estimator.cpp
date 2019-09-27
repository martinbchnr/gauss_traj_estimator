#include "../include/gauss_traj_estimator.h"

GaussTrajEstimator::GaussTrajEstimator()
{
    node_name = ros::this_node::getName();
}

void GaussTrajEstimator::subscribeViaPoints() {
    if (!via_points_topic.empty())
	{
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), goal_pos_topic.c_str());
		via_points_subscriber = node.subscribe(goal_pos_topic,1, &LocalPathPlaner::goal_pos_call_back, this);
	}
	else
	{	
		ROS_INFO("[%s]: Variable '%s' is empty", node_name.c_str(), goal_pos_topic.c_str());
	}
}

void GaussTrajEstimator::subscribeDronePos() {

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
    GaussTrajEstimator gaussian_traj_node;
    gaussian_traj_node.subscribe_to_topics();
    gaussian_traj_node.set_publishers();
    gaussian_traj_node.spin();
}