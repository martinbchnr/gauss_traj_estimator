# gauss_traj_estimator

____

This package contains a ROS implementation (C++) of a Gaussian process regression predicting the future trajectory of an unknown target in intricate dense 3D-environments based on sampling-rejection principle for exploration.

This package utilizes the C++ libraries roscpp, Eigen, octomap as well es dynamicEDT-3D:

- eigen3-library (C++)
- ros-melodic-octomap
- ros-melodic-octomap-msgs
- dynamicEDT3D

sudo make install for dynamicEDT3D (non-ROS package)

ros-melodic-octomap-mapping ros-melodic-octomap-ros ros-melodic-octomap-server
