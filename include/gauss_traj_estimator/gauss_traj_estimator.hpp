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
		void posCallBack_1(const geometry_msgs::PoseWithCovarianceStamped msg);
		void SubscribeToTopics();
		void PublishAllocatedTasks();
		std::vector<std::pair<double,double>> convert_goal_list_to_vector_pair(const goal_list::GoalObjectList pos_array);
		geometry_msgs::PoseArray convert_vector_pair_to_ros_array(const std::vector<std::pair<double,double>> vector);

		void spin();

}

