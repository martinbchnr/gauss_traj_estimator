


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include "gp.hpp"

class GaussTrajEstimator {

  std::string node_name;
  ros::NodeHandle node;

  // Subscribers
  std::string target_pose_topic = "/target_pose";
  std::string train_poses_topic = "/train_data_pos";
  std::string train_times_topic = "/train_time";

  ros::Subscriber target_pose_subscriber;
  ros::Subscriber train_poses_subscriber;
  ros::Subscriber train_times_subscriber;

  // Publishers
  ros::Publisher target_pred_path_mean_pub;
  ros::Publisher target_pred_path_cov_pub;


  std::string target_pred_path_mean_topic = "/target_pred_path_mean";
  std::string target_pred_path_cov_topic = "/target_pred_path_cov";

  // Variables to store subscribed data
  geometry_msgs::PoseWithCovarianceStamped target_pose;
  geometry_msgs::PoseArray train_poses;
  std_msgs::Float32MultiArray train_times;

  // Variables to store generated data
  geometry_msgs::PoseArray pred_path_mean;
  std_msgs::Float32MultiArray pred_path_cov;
  

  // Conversion methods between ROS messages and Eigen data types
  std_msgs::Float32MultiArray EigenToRosSigmaArray(const Eigen::MatrixXd sigma_matrix);
  std_msgs::Float32MultiArray EigenToRosTimeArray(const Eigen::MatrixXd time_matrix);
  geometry_msgs::PoseArray EigenToRosPoseArray(const Eigen::MatrixXd matrix);
  Eigen::MatrixXd RosPoseToEigenArray(const geometry_msgs::PoseArray pos_array);


  public:
  GaussTrajEstimator();
  ~GaussTrajEstimator();
  void targetPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg);
  void trainPosesCallback(const geometry_msgs::PoseArray msg);
  void trainTimesCallback(const std_msgs::Float32MultiArray msg);
  void SubscribeTrainPoses();
  void SubscribeTrainTimes();
  void SubscribeTargetPose();
  void PublishPredictions();
  void spin();
};

