#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include "regressor.hpp"

class GaussTrajEstimator {

    std::string node_name;
    ros::NodeHandle node;

    // Subscribers

    // Publishers

    // Vars to store subscribed data

    public:
		GaussTrajEstimator();
		~GaussTrajEstimator();
		void targetPosCallback(const geometry_msgs::PoseWithCovarianceStamped msg);
		void SubscribeToTopics();
		void PublishPredictions();
		void spin();
}

