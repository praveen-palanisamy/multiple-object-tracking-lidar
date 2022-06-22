#include "kf_tracker/CKalmanFilter.h"
#include "kf_tracker/featureDetection.h"
#include "opencv2/video/tracking.hpp"
#include "pcl_ros/point_cloud.h"
#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <string.h>

#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <limits>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace cv;

ros::Publisher objID_pub;

// KF init
const int clusterMax = 50;
int stateDim = 4; // [x,y,v_x,v_y]//,w,h]
int measDim = 2;  // [z_x,z_y,z_w,z_h]
int ctrlDim = 0;
std::vector<cv::KalmanFilter> KFS;

ros::Publisher pub_cluster0;
ros::Publisher pub_cluster1;
ros::Publisher pub_cluster2;
ros::Publisher pub_cluster3;
ros::Publisher pub_cluster4;
ros::Publisher pub_cluster5;

ros::Publisher markerPub;

std::vector<geometry_msgs::Point> prevClusterCenters;

cv::Mat state(stateDim, 1, CV_32F);
cv::Mat_<float> measurement(2, 1);

std::vector<int> objID; // Output of the data association using KF
                        // measurement.setTo(Scalar(0));

bool firstFrame = true;

// calculate euclidean distance of two points
double euclidean_distance(geometry_msgs::Point &p1, geometry_msgs::Point &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z));
}
/*
//Count unique object IDs. just to make sure same ID has not been assigned to
two KF_Trackers. int countIDs(vector<int> v)
{
    transform(v.begin(), v.end(), v.begin(), abs); // O(n) where n =
distance(v.end(), v.begin()) sort(v.begin(), v.end()); // Average case O(n log
n), worst case O(n^2) (usually implemented as quicksort.
    // To guarantee worst case O(n log n) replace with make_heap, then
sort_heap.

    // Unique will take a sorted range, and move things around to get duplicated
    // items to the back and returns an iterator to the end of the unique
section of the range auto unique_end = unique(v.begin(), v.end()); // Again n
comparisons return distance(unique_end, v.begin()); // Constant time for random
access iterators (like vector's)
}
*/

/*

objID: vector containing the IDs of the clusters that should be associated with
each KF_Tracker objID[0] corresponds to KFT0, objID[1] corresponds to KFT1 etc.
*/

std::pair<int, int> findIndexOfMin(std::vector<std::vector<float>> distMat) {
  cout << "findIndexOfMin cALLED\n";
  std::pair<int, int> minIndex;
  float minEl = std::numeric_limits<float>::max();
  cout << "minEl=" << minEl << "\n";
  for (int i = 0; i < distMat.size(); i++)
    for (int j = 0; j < distMat.at(0).size(); j++) {
      if (distMat[i][j] < minEl) {
        minEl = distMat[i][j];
        minIndex = std::make_pair(i, j);
      }
    }
  cout << "minIndex=" << minIndex.first << "," << minIndex.second << "\n";
  return minIndex;
}
void KFT(const std_msgs::Float32MultiArray ccs) {

  // First predict, to update the internal statePre variable

  // cout<<"Pred successfull\n";

  // cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
  // cout<<"Prediction 1
  // ="<<prediction.at<float>(0)<<","<<prediction.at<float>(1)<<"\n";

  // Get measurements
  // Extract the position of the clusters forom the multiArray. To check if the
  // data coming in, check the .z (every third) coordinate and that will be 0.0
  std::vector<geometry_msgs::Point> clusterCenters; // clusterCenters

  int i = 0;
  for (std::vector<float>::const_iterator it = ccs.data.begin();
       it != ccs.data.end(); it += 3) {
    geometry_msgs::Point pt;
    pt.x = *it;
    pt.y = *(it + 1);
    pt.z = *(it + 2);

    clusterCenters.push_back(pt);
  }

  //  cout<<"CLusterCenters Obtained"<<"\n";
  std::vector<geometry_msgs::Point> KFpredictions;
  i = 0;
  for (auto& KF: KFS) {
    cv::Mat pred = KF.predict();
    geometry_msgs::Point pt;
    pt.x = pred.at<float>(0);
    pt.y = pred.at<float>(1);
    pt.z = pred.at<float>(2);

    KFpredictions.push_back(pt);
  }
  // cout<<"Got predictions"<<"\n";

  // Find the cluster that is more probable to be belonging to a given KF.
  objID.clear();   // Clear the objID vector
  objID.resize(clusterMax); // Allocate default elements so that [i] doesnt segfault.
                   // Should be done better
  // Copy clusterCentres for modifying it and preventing multiple assignments of
  // the same ID
  std::vector<geometry_msgs::Point> copyOfClusterCenters(clusterCenters);
  std::vector<std::vector<float>> distMat;

  for (int filterN = 0; filterN < clusterMax; filterN++) {
    std::vector<float> distVec;
    for (int n = 0; n < clusterMax; n++) {
      distVec.push_back(
          euclidean_distance(KFpredictions[filterN], copyOfClusterCenters[n]));
    }

    distMat.push_back(distVec);
    /*// Based on distVec instead of distMat (global min). Has problems with the
    person's leg going out of scope int
    ID=std::distance(distVec.begin(),min_element(distVec.begin(),distVec.end()));
     //cout<<"finterlN="<<filterN<<"   minID="<<ID
     objID.push_back(ID);
    // Prevent assignment of the same object ID to multiple clusters
     copyOfClusterCenters[ID].x=100000;// A large value so that this center is
    not assigned to another cluster copyOfClusterCenters[ID].y=10000;
     copyOfClusterCenters[ID].z=10000;
    */
    cout << "filterN=" << filterN << "\n";
  }

  cout << "distMat.size()" << distMat.size() << "\n";
  cout << "distMat[0].size()" << distMat.at(0).size() << "\n";
  // DEBUG: print the distMat
  for (const auto &row : distMat) {
    for (const auto &s : row)
      std::cout << s << ' ';
    std::cout << std::endl;
  }

  for (int clusterCount = 0; clusterCount < clusterMax; clusterCount++) {
    // 1. Find min(distMax)==> (i,j);
    std::pair<int, int> minIndex(findIndexOfMin(distMat));
    cout << "Received minIndex=" << minIndex.first << "," << minIndex.second
         << "\n";
    // 2. objID[i]=clusterCenters[j]; counter++
    objID[minIndex.first] = minIndex.second;

    // 3. distMat[i,:]=10000; distMat[:,j]=10000
    distMat[minIndex.first] =
        std::vector<float>(clusterMax, 10000.0); // Set the row to a high number.
    for (int row = 0; row < distMat.size();
         row++) // set the column to a high number
    {
      distMat[row][minIndex.second] = 10000.0;
    }
    // 4. if(counter<clusterMax) got to 1.
    cout << "clusterCount=" << clusterCount << "\n";
  }

  // cout<<"Got object IDs"<<"\n";
  // countIDs(objID);// for verif/corner cases

  // display objIDs
  /* DEBUG
    cout<<"objID= ";
    for(auto it=objID.begin();it!=objID.end();it++)
        cout<<*it<<" ,";
    cout<<"\n";
    */

  visualization_msgs::MarkerArray clusterMarkers;

  for (int i = 0; i < clusterMax; i++) {
    visualization_msgs::Marker m;

    m.id = i;
    m.type = visualization_msgs::Marker::CUBE;
    m.header.frame_id = "map";
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
    m.action = visualization_msgs::Marker::ADD;
    m.color.a = 1.0;
    m.color.r = i % 2 ? 1 : 0;
    m.color.g = i % 3 ? 1 : 0;
    m.color.b = i % 4 ? 1 : 0;

    // geometry_msgs::Point clusterC(clusterCenters.at(objID[i]));
    geometry_msgs::Point clusterC(KFpredictions[i]);
    m.pose.position.x = clusterC.x;
    m.pose.position.y = clusterC.y;
    m.pose.position.z = clusterC.z;

    clusterMarkers.markers.push_back(m);
  }

  prevClusterCenters = clusterCenters;

  markerPub.publish(clusterMarkers);

  std_msgs::Int32MultiArray obj_id;
  for (auto it = objID.begin(); it != objID.end(); it++)
    obj_id.data.push_back(*it);
  // Publish the object IDs
  objID_pub.publish(obj_id);
  // convert clusterCenters from geometry_msgs::Point to floats
  for (int i = 0; i < clusterMax; i++) {
    vector<float> pt;
    pt.push_back(clusterCenters[objID[i]].x);
    pt.push_back(clusterCenters[objID[i]].y);
    pt.push_back(clusterCenters[objID[i]].z);

    float meas[2] = {pt.at(0), pt.at(1)};
    // The update phase
    cv::Mat measMat = cv::Mat(2, 1, CV_32F, meas);
    if (!(measMat.at<float>(0, 0) == 0.0f || measMat.at<float>(1, 0) == 0.0f))
      Mat estimated = KFS[i].correct(measMat);
  }
  // cout<<"cc[5][0]="<<cc[5].at(0)<<"cc[5][1]="<<cc[5].at(1)<<"cc[5][2]="<<cc[5].at(2)<<"\n";

  // cout<<"meas0Mat"<<meas0Mat<<"\n";

  // Publish the point clouds belonging to each clusters

  // cout<<"estimate="<<estimated.at<float>(0)<<","<<estimated.at<float>(1)<<"\n";
  // Point statePt(estimated.at<float>(0),estimated.at<float>(1));
  // cout<<"DONE KF_TRACKER\n";
}
void publish_cloud(ros::Publisher &pub,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
  sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cluster, *clustermsg);
  clustermsg->header.frame_id = "map";
  clustermsg->header.stamp = ros::Time::now();
  pub.publish(*clustermsg);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)

{
  // cout<<"IF firstFrame="<<firstFrame<<"\n";
  // If this is the first frame, initialize kalman filters for the clustered
  // objects
  if (firstFrame) {
    // Initialize clusterMax Kalman Filters; Assuming clusterMax max objects in the dataset.
    // Could be made generic by creating a Kalman Filter only when a new object
    // is detected

    float dvx = 0.01f; // 1.0
    float dvy = 0.01f; // 1.0
    float dx = 1.0f;
    float dy = 1.0f;

    float sigmaP = 0.01;
    float sigmaQ = 0.1;

    for (auto& KF: KFS) {
      KF.transitionMatrix = (Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                            dvx, 0, 0, 0, 0, dvy);
      cv::setIdentity(KF.measurementMatrix);
      // Process Noise Covariance Matrix Q
      // [ Ex 0  0    0 0    0 ]
      // [ 0  Ey 0    0 0    0 ]
      // [ 0  0  Ev_x 0 0    0 ]
      // [ 0  0  0    1 Ev_y 0 ]
      //// [ 0  0  0    0 1    Ew ]
      //// [ 0  0  0    0 0    Eh ]
      setIdentity(KF.processNoiseCov, Scalar::all(sigmaP));
      // Meas noise cov matrix R
      cv::setIdentity(KF.measurementNoiseCov, cv::Scalar(sigmaQ)); // 1e-1
    }

    // Process the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    /* Creating the KdTree from input point cloud*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *input_cloud);

    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.08);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract(cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    // Vector of cluster pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
    // Cluster centroids
    std::vector<pcl::PointXYZ> clusterCentroids;

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      float x = 0.0;
      float y = 0.0;
      int numPts = 0;
      for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

        cloud_cluster->points.push_back(input_cloud->points[*pit]);
        x += input_cloud->points[*pit].x;
        y += input_cloud->points[*pit].y;
        numPts++;

        // dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
        //                                          origin);
        // mindist_this_cluster = std::min(dist_this_point,
        // mindist_this_cluster);
      }

      pcl::PointXYZ centroid;
      centroid.x = x / numPts;
      centroid.y = y / numPts;
      centroid.z = 0.0;

      cluster_vec.push_back(cloud_cluster);

      // Get the centroid of the cluster
      clusterCentroids.push_back(centroid);
    }

    // Ensure at least clusterMax clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < clusterMax) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
      cluster_vec.push_back(empty_cluster);
    }

    while (clusterCentroids.size() < clusterMax) {
      pcl::PointXYZ centroid;
      centroid.x = 0.0;
      centroid.y = 0.0;
      centroid.z = 0.0;

      clusterCentroids.push_back(centroid);
    }

    // Set initial state
    for (int i = 0; i < clusterMax; ++i) {
      KFS[i].statePre.at<float>(0) = clusterCentroids.at(i).x;
      KFS[i].statePre.at<float>(1) = clusterCentroids.at(i).y;
      KFS[i].statePre.at<float>(2) = 0; // initial v_x
      KFS[i].statePre.at<float>(3) = 0; // initial v_y
    }

    firstFrame = false;

    for (int i = 0; i < clusterMax; i++) {
      geometry_msgs::Point pt;
      pt.x = clusterCentroids.at(i).x;
      pt.y = clusterCentroids.at(i).y;
      prevClusterCenters.push_back(pt);
    }
    /*  // Print the initial state of the kalman filter for debugging
      cout<<"KF0.satePre="<<KF0.statePre.at<float>(0)<<","<<KF0.statePre.at<float>(1)<<"\n";
      cout<<"KF1.satePre="<<KF1.statePre.at<float>(0)<<","<<KF1.statePre.at<float>(1)<<"\n";
      cout<<"KF2.satePre="<<KF2.statePre.at<float>(0)<<","<<KF2.statePre.at<float>(1)<<"\n";
      cout<<"KF3.satePre="<<KF3.statePre.at<float>(0)<<","<<KF3.statePre.at<float>(1)<<"\n";
      cout<<"KF4.satePre="<<KF4.statePre.at<float>(0)<<","<<KF4.statePre.at<float>(1)<<"\n";
      cout<<"KF5.satePre="<<KF5.statePre.at<float>(0)<<","<<KF5.statePre.at<float>(1)<<"\n";

      //cin.ignore();// To be able to see the printed initial state of the
      KalmanFilter
      */
  }

  else {
    // cout<<"ELSE firstFrame="<<firstFrame<<"\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    /* Creating the KdTree from input point cloud*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *input_cloud);

    tree->setInputCloud(input_cloud);

    /* Here we are creating a vector of PointIndices, which contains the actual
     * index information in a vector<int>. The indices of each detected cluster
     * are saved here. Cluster_indices is a vector containing one instance of
     * PointIndices for each detected cluster. Cluster_indices[0] contain all
     * indices of the first cluster in input point cloud.
     */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    // cout<<"PCL init successfull\n";
    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract(cluster_indices);
    // cout<<"PCL extract successfull\n";
    /* To separate each cluster out of the vector<PointIndices> we have to
     * iterate through cluster_indices, create a new PointCloud for each
     * entry and write all points of the current cluster in the PointCloud.
     */
    // pcl::PointXYZ origin (0,0,0);
    // float mindist_this_cluster = 1000;
    // float dist_this_point = 1000;

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    // Vector of cluster pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;

    // Cluster centroids
    std::vector<pcl::PointXYZ> clusterCentroids;

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      float x = 0.0;
      float y = 0.0;
      int numPts = 0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

        cloud_cluster->points.push_back(input_cloud->points[*pit]);

        x += input_cloud->points[*pit].x;
        y += input_cloud->points[*pit].y;
        numPts++;

        // dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
        //                                          origin);
        // mindist_this_cluster = std::min(dist_this_point,
        // mindist_this_cluster);
      }

      pcl::PointXYZ centroid;
      centroid.x = x / numPts;
      centroid.y = y / numPts;
      centroid.z = 0.0;

      cluster_vec.push_back(cloud_cluster);

      // Get the centroid of the cluster
      clusterCentroids.push_back(centroid);
    }
    // cout<<"cluster_vec got some clusters\n";

    // Ensure at least clusterMax clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < clusterMax) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
      cluster_vec.push_back(empty_cluster);
    }

    while (clusterCentroids.size() < clusterMax) {
      pcl::PointXYZ centroid;
      centroid.x = 0.0;
      centroid.y = 0.0;
      centroid.z = 0.0;

      clusterCentroids.push_back(centroid);
    }

    std_msgs::Float32MultiArray cc;
    for (int i = 0; i < clusterMax; i++) {
      cc.data.push_back(clusterCentroids.at(i).x);
      cc.data.push_back(clusterCentroids.at(i).y);
      cc.data.push_back(clusterCentroids.at(i).z);
    }
    // cout<<"clusterMax clusters initialized\n";

    // cc_pos.publish(cc);// Publish cluster mid-points.
    KFT(cc);
    int i = 0;
    bool publishedCluster[clusterMax];
    for (auto it = objID.begin(); it != objID.end();
         it++) { // cout<<"Inside the for loop\n";

      switch (i) {
        cout << "Inside the switch case\n";
      case 0: {
        publish_cloud(pub_cluster0, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 1: {
        publish_cloud(pub_cluster1, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 2: {
        publish_cloud(pub_cluster2, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 3: {
        publish_cloud(pub_cluster3, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 4: {
        publish_cloud(pub_cluster4, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }

      case 5: {
        publish_cloud(pub_cluster5, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      default:
        break;
      }
    }
  }
}

int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "kf_tracker");
  ros::NodeHandle nh;

  for (int i = 0; i < clusterMax; ++i) {
    KFS.push_back(cv::KalmanFilter(stateDim, measDim, ctrlDim, CV_32F));
  }

  // Publishers to publish the state of the objects (pos and vel)
  // objState1=nh.advertise<geometry_msgs::Twist> ("obj_1",1);

  cout << "About to setup callback\n";

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("filtered_cloud", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub_cluster0 = nh.advertise<sensor_msgs::PointCloud2>("cluster_0", 1);
  pub_cluster1 = nh.advertise<sensor_msgs::PointCloud2>("cluster_1", 1);
  pub_cluster2 = nh.advertise<sensor_msgs::PointCloud2>("cluster_2", 1);
  pub_cluster3 = nh.advertise<sensor_msgs::PointCloud2>("cluster_3", 1);
  pub_cluster4 = nh.advertise<sensor_msgs::PointCloud2>("cluster_4", 1);
  pub_cluster5 = nh.advertise<sensor_msgs::PointCloud2>("cluster_5", 1);
  // Subscribe to the clustered pointclouds
  // ros::Subscriber c1=nh.subscribe("ccs",100,KFT);
  objID_pub = nh.advertise<std_msgs::Int32MultiArray>("obj_id", 1);
  /* Point cloud clustering
   */

  // cc_pos=nh.advertise<std_msgs::Float32MultiArray>("ccs",100);//clusterCenter1
  markerPub = nh.advertise<visualization_msgs::MarkerArray>("viz", 1);

  /* Point cloud clustering
   */

  ros::spin();
}
