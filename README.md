# Multiple objects detection, tracking and classification from LIDAR scans/point-clouds


[![DOI](https://zenodo.org/badge/47581608.svg)](https://zenodo.org/badge/latestdoi/47581608)

![Sample demo of multiple object tracking using LIDAR scans](https://media.giphy.com/media/3YKG95w9gu263yQwDa/giphy.gif)

PCL based ROS package to Detect/Cluster --> Track --> Classify static and dynamic objects in real-time from LIDAR scans implemented in C++.

### Features:

- K-D tree based point cloud processing for object feature detection from point clouds
- Unsupervised euclidean cluster extraction (3D) or k-means clustering based on detected features and refinement using RANSAC (2D)
- Stable tracking (object ID & data association) with an ensemble of Kalman Filters 
- Robust compared to k-means clustering with mean-flow tracking

### Usage:

Follow the steps below to use this (`multi_object_tracking_lidar`) package:

1. [Create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) (if you do not have one setup already). 
1. Navigate to the `src` folder in your catkin workspace: `cd ~/catkin_ws/src`
1. Clone this repository: `git clone https://github.com/praveen-palanisamy/multiple-object-tracking-lidar.git`
1. Compile and build the package: `cd ~/catkin_ws && catkin_make`
1. Add the catkin workspace to your ROS environment: `source ~/catkin_ws/devel/setup.bash`
1. Run the `kf_tracker` ROS node in this package: `rosrun multi_object_tracking_lidar kf_tracker`

If all went well, the ROS node should be up and running! As long as you have the point clouds published on to the `filtered_cloud` rostopic, you should see outputs from this node published onto the `obj_id`, `cluster_0`, `cluster_1`, …, `cluster_5` topics along with the markers on `viz` topic which you can visualize using RViz.

### Supported point-cloud streams/sources:
The input point-clouds can be from:
1. A real LiDAR or 
2. A simulated LiDAR or 
3. A point cloud dataset or 
4. Any other data source that produces point clouds

## Citing

If you use the code or snippets from this repository in your work, please cite:

```bibtex
@software{praveen_palanisamy_2019_3559187,
  author       = {Praveen Palanisamy},
  title        = {{praveen-palanisamy/multiple-object-tracking-lidar: 
                   Multiple-Object-Tracking-from-Point-Clouds_v1.0.2}},
  month        = dec,
  year         = 2019,
  publisher    = {Zenodo},
  version      = {1.0.2},
  doi          = {10.5281/zenodo.3559187},
  url          = {https://doi.org/10.5281/zenodo.3559186}
}
```

### Wiki

[Checkout the Wiki pages](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar/wiki)

1. [Multiple-object tracking from pointclouds using a Velodyne VLP-16](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar/wiki/velodyne_vlp16)
