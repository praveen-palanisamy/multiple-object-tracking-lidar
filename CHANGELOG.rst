^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multi_object_tracking_lidar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2020-06-27)
------------------
* Merge pull request #26 from artursg/noetic-devel
  C++11 --> C++14 to allow compiling with later versions of PCL, ROS Neotic
* Compiles under ROS Noetic
* Merge pull request #25 from praveen-palanisamy/add-license-1
  Add MIT LICENSE
* Add LICENSE
* Merge pull request #24 from mzahran001/patch-1
  Fix broken hyperlink to wiki page in README
* Fixing link error
* Updated README to make clustering approach for 3D vs 2D clear #21
* Added DOI and citing info
* Contributors: Artur Sagitov, Mohamed Zahran, Praveen Palanisamy

1.0.2 (2019-12-01)
------------------
* Added link to wiki pages
* Updated readme with suuported pointcloud sources
* Updated README to clarify real, sim, dataset LiDAR data
* Contributors: Praveen Palanisamy

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
