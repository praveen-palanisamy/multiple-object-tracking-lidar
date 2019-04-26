^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multi_object_tracking_lidar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2019-04-26)
------------------
* Fixed cv_bridge build depend
* Removed indirection op to be compatible with OpenCV 3+
* Added visualization_msgs & cv_bridge build & run dependencies
* Contributors: Praveen Palanisamy

1.0.0 (2019-04-13)
------------------
* Updated README with usage instructions
* Renamed node name to kf_tracker to match bin name
* Changed package name to multi_object_tracking_lidar
* Updated package info & version num
* Updated with a short demo on sample AV LIDAR scans
* Added README with a short summary of the code
* Working state of Multiple object stable tracking using Lidar scans with an extended Kalman Filter (rosrun kf_tracker tracker). A naive tracker is implemented in main_naive.cpp for comparison (rosrun kf_tracker naive_tracker).
* v2. Unsupervised clustering is incorporated into the same node (tracker).
* v1. Object ID/data association works. In this version PCL based unsupervised clustering is done separately.
* Contributors: Praveen Palanisamy
