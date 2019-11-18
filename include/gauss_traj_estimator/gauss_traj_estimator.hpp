


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "gauss_traj_estimator/TrainTimes.h"
#include "gp.hpp"

class GaussTrajEstimator {

  std::string node_name;
  ros::NodeHandle node;

  // Subscribers
  std::string target_pose_topic = "/target_pose";
  std::string train_poses_topic = "/train_poses";
  std::string train_times_topic = "/train_times";

  ros::Subscriber target_pose_subscriber;
  ros::Subscriber train_poses_subscriber;
  ros::Subscriber train_times_subscriber;

  // Publishers
  ros::Publisher target_pred_path_mean_pub;
  ros::Publisher target_pred_path_cov_pub;
  ros::Publisher evaltd_training_points_pub;

  std::string target_pred_path_mean_topic = "/target_pred_path_mean";
  std::string target_pred_path_cov_topic = "/target_pred_path_cov";
  std::string evaltd_training_points_topic = "/evaltd_training_points";


  // ROS message variables to store subscribed data
  geometry_msgs::PoseWithCovarianceStamped target_pose_rosmsg;
  geometry_msgs::PoseArray train_poses_rosmsg;
  gauss_traj_estimator::TrainTimes train_times_rosmsg;

  // Internal variables for computation
  Eigen::MatrixXd target_pose;
   
  Eigen::MatrixXd train_times;
  Eigen::MatrixXd train_times_x;
	Eigen::MatrixXd train_times_y;

  Eigen::MatrixXd train_poses;
  Eigen::MatrixXd train_poses_x;
	Eigen::MatrixXd train_poses_y;

  // ROS message variables to store generated data
  nav_msgs::Path pred_path_mean_rosmsg;
  visualization_msgs::MarkerArray evaltd_training_points_rosmsg;
  std_msgs::Float32MultiArray pred_path_cov_rosmsg;
  

  public:
  GaussTrajEstimator();
  ~GaussTrajEstimator();
  void targetPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg);
  void trainPosesCallback(const geometry_msgs::PoseArray msg);
  void trainTimesCallback(const gauss_traj_estimator::TrainTimes msg);
  
  // Message and communication methods
  void SubscribeTrainPoses();
  void SubscribeTrainTimes();
  void SubscribeTargetPose();
  void PublishPredictions();
  void PublishTrainingData();
  void spin();

  // Conversion methods between ROS messages and Eigen data types
  std_msgs::Float32MultiArray EigenToRosSigmaArray(const Eigen::MatrixXd sigma_matrix);
  std_msgs::Float32MultiArray EigenToRosTimeArray(const Eigen::MatrixXd time_matrix);
  nav_msgs::Path EigenToRosPath(const Eigen::MatrixXd matrix);
  Eigen::MatrixXd RosPosesToEigenArray(const geometry_msgs::PoseArray pos_array);
  Eigen::MatrixXd RosPoseWithCovToEigenArray(const geometry_msgs::PoseWithCovarianceStamped pose);
  Eigen::MatrixXd RosTimesToEigenArray(const gauss_traj_estimator::TrainTimes times_array);
  visualization_msgs::MarkerArray EigenToRosMarkerArray(const Eigen::MatrixXd matrix);
};

