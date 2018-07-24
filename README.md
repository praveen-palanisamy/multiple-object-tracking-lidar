# Multiple objects detection, tracking and classification from LIDAR scans/point-clouds
![Sample demo of multiple object tracking using LIDAR scans](https://media.giphy.com/media/3YKG95w9gu263yQwDa/giphy.gif)

PCL based ROS package to Detect/Cluster --> Track --> Classify static and dynamic objects in real-time from LIDAR scans implemented in C++.

### Features:

- K-D tree based point cloud processing for object feature detection from point clouds
- Unsupervised k-means clustering based on detected features and refinement using RANSAC
- Stable tracking (object ID & data association) with an ensemble of Kalman Filters 
- Robust compared to k-means clustering with mean-flow tracking
