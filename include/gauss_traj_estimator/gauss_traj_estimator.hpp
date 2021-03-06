


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "gauss_traj_estimator/TrainTimes.h"
#include <sensor_msgs/PointCloud.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeNode.h>
#include <octomap/octomap_types.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include "gp.hpp" // inherits gaussian.hpp-class
#include "path_eval.hpp"

typedef unsigned int uint;


class GaussTrajEstimator {

  std::string node_name;
  //ros::NodeHandle node;

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
  ros::Publisher sampled_pred_paths_pub;
  ros::Publisher valid_sampled_pred_paths_pub;
  ros::Publisher valid_pred_path_mean_pub;
  ros::Publisher edf_field_pub;
  ros::Publisher valid_pred_path_cov_pos_pub;
  ros::Publisher valid_pred_path_cov_neg_pub;
  

  std::string target_pred_path_mean_topic = "/target_pred_path_mean";
  std::string target_pred_path_cov_topic = "/target_pred_path_cov";
  std::string evaltd_training_points_topic = "/evaltd_training_points";
  std::string sampled_pred_paths_topic = "/sampled_pred_paths";
  std::string valid_sampled_pred_paths_topic = "/valid_sampled_pred_paths";
  std::string valid_pred_path_mean_topic = "/valid_pred_path_mean";
  std::string edf_field_topic = "/edf_field";
  std::string valid_pred_path_cov_pos_topic = "/valid_pred_path_cov_pos";
  std::string valid_pred_path_cov_neg_topic = "/valid_pred_path_cov_neg";

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
  visualization_msgs::MarkerArray sampled_pred_path_rosmsg;
  visualization_msgs::MarkerArray valid_sampled_pred_path_rosmsg;
  std_msgs::Float32MultiArray pred_path_cov_rosmsg;

  nav_msgs::Path valid_mean_path_rosmsg;
  nav_msgs::Path valid_pred_path_cov_pos_rosmsg;
  nav_msgs::Path valid_pred_path_cov_neg_rosmsg;
  nav_msgs::Path latest_valid_mean_path_rosmsg;


  struct GP_PARAMS {
    double signal_var; // done
    double length_scale; // done
    double noise_var; // done
    double start_t; // done
    double end_t;  // done
    int test_dim_path;  // done    
  };

  struct TRAINING_DATA_PARAMS {
    std::vector<double> X_train_x; // done
    std::vector<double> X_train_y; // done
    std::vector<double> X_train_z; // done
    std::vector<double> t_train;  // done
    int dim; // done
    int num_samples; // done
  };

  struct GAUSSIAN_PARAMS {
    double norm_var; // done
    double uniform_lower_bound; // not used yet
    double uniform_upper_bound;   // not used yet
    string sampling_mode; // not used yet
  };

  struct SAMPLING_PARAMS {
    int sample_count;    // done
  };

  struct PATH_COST_PARAMS {
    double r_safe; // done
    double ground_rejection_height;  // don 
  };

  struct NODE_PARAMS {
    string frame_id; // done
    double run_freq;  // done
    int map_res_scaler; //done
    string map_file; //done
  };

 

  GP_PARAMS gp_params;
	TRAINING_DATA_PARAMS training_data;
	GAUSSIAN_PARAMS gaussian_params;
	SAMPLING_PARAMS sampling_params;
	PATH_COST_PARAMS path_cost_params;
	NODE_PARAMS node_params;
  

  public:
  GaussTrajEstimator();
  ~GaussTrajEstimator();
  void targetPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg);
  void trainPosesCallback(const geometry_msgs::PoseArray msg);
  void trainTimesCallback(const gauss_traj_estimator::TrainTimes msg);
  
  ros::NodeHandle node;

  // Message and communication methods
  void SubscribeTrainPoses();
  void SubscribeTrainTimes();
  void SubscribeTargetPose();
  void PublishPredictions();
  void PublishTrainingData();
  void PublishSampledData();
  void PublishValidData();
  void PublishEDF();
  void spin();
  void GenerateEDF();
  void ReceiveParams(const ros::NodeHandle& nh);

  
  // Conversion methods between ROS messages and Eigen data types
  std_msgs::Float32MultiArray EigenToRosSigmaArray(const Eigen::MatrixXd sigma_matrix);
  std_msgs::Float32MultiArray EigenToRosTimeArray(const Eigen::MatrixXd time_matrix);
  nav_msgs::Path EigenToRosPath(const Eigen::MatrixXd matrix);
  Eigen::MatrixXd RosPosesToEigenArray(const geometry_msgs::PoseArray pos_array);
  Eigen::MatrixXd RosPoseWithCovToEigenArray(const geometry_msgs::PoseWithCovarianceStamped pose);
  Eigen::MatrixXd RosTimesToEigenArray(const gauss_traj_estimator::TrainTimes times_array);
  visualization_msgs::MarkerArray EigenToRosMarkerArray(const Eigen::MatrixXd matrix);
  visualization_msgs::MarkerArray EigenToRosSampledPathsMarkerArray(const Eigen::MatrixXd matrix, const uint sample_count);
  visualization_msgs::MarkerArray EigenToRosSampledPathsMarkerArrayColored(const Eigen::MatrixXd matrix, const uint sample_count, const Eigen::MatrixXd intensity);

  PathEvaluator path_cost_eval;
};

