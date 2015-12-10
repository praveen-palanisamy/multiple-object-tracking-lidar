#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>
#include "kf_tracker/featureDetection.h"
#include "kf_tracker/CKalmanFilter.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/video/tracking.hpp"
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;
using namespace cv;


ros::Publisher objID_pub;

    // KF init
    int stateDim=4;// [x,y,v_x,v_y]//,w,h]
    int measDim=2;// [z_x,z_y,z_w,z_h]
    int ctrlDim=0;
    cv::KalmanFilter KF0(stateDim,measDim,ctrlDim,CV_32F);
    cv::KalmanFilter KF1(stateDim,measDim,ctrlDim,CV_32F);
    cv::KalmanFilter KF2(stateDim,measDim,ctrlDim,CV_32F);
    cv::KalmanFilter KF3(stateDim,measDim,ctrlDim,CV_32F);
    cv::KalmanFilter KF4(stateDim,measDim,ctrlDim,CV_32F);
    cv::KalmanFilter KF5(stateDim,measDim,ctrlDim,CV_32F);

    ros::Publisher pub_cluster0;
    ros::Publisher pub_cluster1;
    ros::Publisher pub_cluster2;
    ros::Publisher pub_cluster3;
    ros::Publisher pub_cluster4;
    ros::Publisher pub_cluster5;

    std::vector<geometry_msgs::Point> prevClusterCenters;


    cv::Mat state(stateDim,1,CV_32F);
    cv::Mat_<float> measurement(2,1); 

    std::vector<int> objID;// Output of the data association using KF
   // measurement.setTo(Scalar(0));

bool firstFrame=true;

// calculate euclidean distance of two points
  double euclidean_distance(geometry_msgs::Point& p1, geometry_msgs::Point& p2)
  {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
  }
/*
//Count unique object IDs. just to make sure same ID has not been assigned to two KF_Trackers.  
int countIDs(vector<int> v)
{
    transform(v.begin(), v.end(), v.begin(), abs); // O(n) where n = distance(v.end(), v.begin())
    sort(v.begin(), v.end()); // Average case O(n log n), worst case O(n^2) (usually implemented as quicksort.
    // To guarantee worst case O(n log n) replace with make_heap, then sort_heap.

    // Unique will take a sorted range, and move things around to get duplicated
    // items to the back and returns an iterator to the end of the unique section of the range
    auto unique_end = unique(v.begin(), v.end()); // Again n comparisons
    return distance(unique_end, v.begin()); // Constant time for random access iterators (like vector's)
}
*/

/*

objID: vector containing the IDs of the clusters that should be associated with each KF_Tracker
objID[0] corresponds to KFT0, objID[1] corresponds to KFT1 etc.
*/
void KFT(const std_msgs::Float32MultiArray ccs)
{



    // First predict, to update the internal statePre variable

    std::vector<cv::Mat> pred{KF0.predict(),KF1.predict(),KF2.predict(),KF3.predict(),KF4.predict(),KF5.predict()};
 //cout<<"Pred successfull\n";

    //cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
   // cout<<"Prediction 1 ="<<prediction.at<float>(0)<<","<<prediction.at<float>(1)<<"\n";

    // Get measurements
    // Extract the position of the clusters forom the multiArray. To check if the data
    // coming in, check the .z (every third) coordinate and that will be 0.0
    std::vector<geometry_msgs::Point> clusterCenters;//clusterCenters
   
    int i=0;
    for (std::vector<float>::const_iterator it=ccs.data.begin();it!=ccs.data.end();it+=3)
    {
        geometry_msgs::Point pt;
        pt.x=*it;
        pt.y=*(it+1);
        pt.z=*(it+2);

        clusterCenters.push_back(pt);
       
    }

  //  cout<<"CLusterCenters Obtained"<<"\n";
    std::vector<geometry_msgs::Point> KFpredictions;
    i=0;
    for (auto it=pred.begin();it!=pred.end();it++)
    {
        geometry_msgs::Point pt;
        pt.x=(*it).at<float>(0);
        pt.y=(*it).at<float>(1);
        pt.z=(*it).at<float>(2);

        KFpredictions.push_back(pt);
        
    }
   // cout<<"Got predictions"<<"\n";

    /* Original Version using Kalman filter prediction
    
    // Find the cluster that is more probable to be belonging to a given KF.
    objID.clear();//Clear the objID vector
    for(int filterN=0;filterN<6;filterN++)
    {
        std::vector<float> distVec;
        for(int n=0;n<6;n++)
            distVec.push_back(euclidean_distance(KFpredictions[filterN],clusterCenters[n]));
       
      // cout<<"distVec[="<<distVec[0]<<","<<distVec[1]<<","<<distVec[2]<<","<<distVec[3]<<","<<distVec[4]<<","<<distVec[5]<<"\n";
        objID.push_back(std::distance(distVec.begin(),min_element(distVec.begin(),distVec.end())));
       // cout<<"MinD for filter"<<filterN<<"="<<*min_element(distVec.begin(),distVec.end())<<"\n";
    
    }

   // cout<<"Got object IDs"<<"\n";
    //countIDs(objID);// for verif/corner cases
      Original version using kalman filter prediction
    */
    //display objIDs
  /* DEBUG
    cout<<"objID= ";
    for(auto it=objID.begin();it!=objID.end();it++)
        cout<<*it<<" ,";
    cout<<"\n";
    */

/* Naive version without using kalman filter */
objID.clear();//Clear the objID vector
    for(int filterN=0;filterN<6;filterN++)
    {
        std::vector<float> distVec;
        for(int n=0;n<6;n++)
            distVec.push_back(euclidean_distance(prevClusterCenters[n],clusterCenters[n]));
       
      // cout<<"distVec[="<<distVec[0]<<","<<distVec[1]<<","<<distVec[2]<<","<<distVec[3]<<","<<distVec[4]<<","<<distVec[5]<<"\n";
        objID.push_back(std::distance(distVec.begin(),min_element(distVec.begin(),distVec.end())));
       // cout<<"MinD for filter"<<filterN<<"="<<*min_element(distVec.begin(),distVec.end())<<"\n";
    
    }
     /* Naive version without kalman filter */
    //prevClusterCenters.clear();
    // Set the current associated clusters to the prevClusterCenters
     for (int i=0;i<6;i++)
     {
        prevClusterCenters[objID.at(i)]=clusterCenters.at(i);
       
     }

     /* Naive version without kalman filter */

/* Naive version without using kalman filter */    


    std_msgs::Int32MultiArray obj_id;
    for(auto it=objID.begin();it!=objID.end();it++)
        obj_id.data.push_back(*it);
    objID_pub.publish(obj_id);
    // convert clusterCenters from geometry_msgs::Point to floats
    std::vector<std::vector<float> > cc;
    for (int i=0;i<clusterCenters.size();i++)
    {
        vector<float> pt;
        pt.push_back(clusterCenters[i].x);
        pt.push_back(clusterCenters[i].y);
        pt.push_back(clusterCenters[i].z);
        
        cc.push_back(pt);
    }
    //cout<<"cc[5][0]="<<cc[5].at(0)<<"cc[5][1]="<<cc[5].at(1)<<"cc[5][2]="<<cc[5].at(2)<<"\n";
    float meas0[3]={cc[0].at(0),cc[0].at(1)};
    float meas1[3]={cc[1].at(0),cc[1].at(1)};
    float meas2[3]={cc[2].at(0),cc[2].at(1)};
    float meas3[3]={cc[3].at(0),cc[3].at(1)};
    float meas4[3]={cc[4].at(0),cc[4].at(1)};
    float meas5[3]={cc[5].at(0),cc[5].at(1)};



    // The update phase 
    cv::Mat meas0Mat=cv::Mat(2,1,CV_32F,meas0);
    cv::Mat meas1Mat=cv::Mat(2,1,CV_32F,meas1);
    cv::Mat meas2Mat=cv::Mat(2,1,CV_32F,meas2);
    cv::Mat meas3Mat=cv::Mat(2,1,CV_32F,meas3);
    cv::Mat meas4Mat=cv::Mat(2,1,CV_32F,meas4);
    cv::Mat meas5Mat=cv::Mat(2,1,CV_32F,meas5);

//cout<<"meas0Mat"<<meas0Mat<<"\n";

    Mat estimated0 = KF0.correct(meas0Mat);
    Mat estimated1 = KF0.correct(meas1Mat);
    Mat estimated2 = KF0.correct(meas2Mat);
    Mat estimated3 = KF0.correct(meas3Mat);
    Mat estimated4 = KF0.correct(meas4Mat);
    Mat estimated5 = KF0.correct(meas5Mat);
 
    // Publish the point clouds belonging to each clusters


   // cout<<"estimate="<<estimated.at<float>(0)<<","<<estimated.at<float>(1)<<"\n";
   // Point statePt(estimated.at<float>(0),estimated.at<float>(1));
//cout<<"DONE KF_TRACKER\n";
   
}
void publish_cloud(ros::Publisher& pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster){
  sensor_msgs::PointCloud2::Ptr clustermsg (new sensor_msgs::PointCloud2);
  pcl::toROSMsg (*cluster , *clustermsg);
  clustermsg->header.frame_id = "/map";
  clustermsg->header.stamp = ros::Time::now();
  pub.publish (*clustermsg);

}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)

{
    cout<<"IF firstFrame="<<firstFrame<<"\n";
    // If this is the first frame, initialize kalman filters for the clustered objects
if (firstFrame)
{   
  // Initialize 6 Kalman Filters; Assuming 6 max objects in the dataset. 
  // Could be made generic by creating a Kalman Filter only when a new object is detected  
    KF0.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
    KF1.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
    KF2.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
    KF3.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
    KF4.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
    KF5.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

    cv::setIdentity(KF0.measurementMatrix);
    cv::setIdentity(KF1.measurementMatrix);
    cv::setIdentity(KF2.measurementMatrix);
    cv::setIdentity(KF3.measurementMatrix);
    cv::setIdentity(KF4.measurementMatrix);
    cv::setIdentity(KF5.measurementMatrix);
    // Process Noise Covariance Matrix Q
    // [ Ex 0  0    0 0    0 ]
    // [ 0  Ey 0    0 0    0 ]
    // [ 0  0  Ev_x 0 0    0 ]
    // [ 0  0  0    1 Ev_y 0 ]
    //// [ 0  0  0    0 1    Ew ]
    //// [ 0  0  0    0 0    Eh ]
    setIdentity(KF0.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF1.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF2.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF3.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF4.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF5.processNoiseCov, Scalar::all(1e-4));
    // Meas noise cov matrix R
     cv::setIdentity(KF0.measurementNoiseCov, cv::Scalar(1e-1));
     cv::setIdentity(KF1.measurementNoiseCov, cv::Scalar(1e-1));
     cv::setIdentity(KF2.measurementNoiseCov, cv::Scalar(1e-1));
     cv::setIdentity(KF3.measurementNoiseCov, cv::Scalar(1e-1));
     cv::setIdentity(KF4.measurementNoiseCov, cv::Scalar(1e-1));
     cv::setIdentity(KF5.measurementNoiseCov, cv::Scalar(1e-1));

// Process the point cloud
     pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      /* Creating the KdTree from input point cloud*/
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::fromROSMsg (*input, *input_cloud);

  tree->setInputCloud (input_cloud);

  /* Here we are creating a vector of PointIndices, which contains the actual index
   * information in a vector<int>. The indices of each detected cluster are saved here.
   * Cluster_indices is a vector containing one instance of PointIndices for each detected 
   * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
   */
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.08); 
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (600);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract (cluster_indices);

  /* To separate each cluster out of the vector<PointIndices> we have to 
   * iterate through cluster_indices, create a new PointCloud for each 
   * entry and write all points of the current cluster in the PointCloud. 
   */
  //pcl::PointXYZ origin (0,0,0);
  //float mindist_this_cluster = 1000;
  //float dist_this_point = 1000;

  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  // Vector of cluster pointclouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vec;

  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for(pit = it->indices.begin(); pit != it->indices.end(); pit++) 
          {
          
                  cloud_cluster->points.push_back(input_cloud->points[*pit]);
                  //dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
                  //                                          origin);
                  //mindist_this_cluster = std::min(dist_this_point, mindist_this_cluster);
          }

         
      cluster_vec.push_back(cloud_cluster);

    }

    //Ensure at least 6 clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < 6){
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0,0,0));
      cluster_vec.push_back(empty_cluster);
    }
    

     // Set initial state
     KF0.statePre.at<float>(0)=cluster_vec[0]->points[cluster_vec[0]->points.size()/2].x;
     
     KF0.statePre.at<float>(1)=cluster_vec[0]->points[cluster_vec[0]->points.size()/2].y;
     KF0.statePre.at<float>(2)=0;// initial v_x
     KF0.statePre.at<float>(3)=0;//initial v_y

     // Set initial state
     KF1.statePre.at<float>(0)=cluster_vec[1]->points[cluster_vec[1]->points.size()/2].x;
     KF1.statePre.at<float>(1)=cluster_vec[1]->points[cluster_vec[1]->points.size()/2].y;
     KF1.statePre.at<float>(2)=0;// initial v_x
     KF1.statePre.at<float>(3)=0;//initial v_y

     // Set initial state
     KF2.statePre.at<float>(0)=cluster_vec[2]->points[cluster_vec[2]->points.size()/2].x;
     KF2.statePre.at<float>(1)=cluster_vec[2]->points[cluster_vec[2]->points.size()/2].y;
     KF2.statePre.at<float>(2)=0;// initial v_x
     KF2.statePre.at<float>(3)=0;//initial v_y


     // Set initial state
     KF3.statePre.at<float>(0)=cluster_vec[3]->points[cluster_vec[3]->points.size()/2].x;
     KF3.statePre.at<float>(1)=cluster_vec[3]->points[cluster_vec[3]->points.size()/2].y;
     KF3.statePre.at<float>(2)=0;// initial v_x
     KF3.statePre.at<float>(3)=0;//initial v_y

     // Set initial state
     KF4.statePre.at<float>(0)=cluster_vec[4]->points[cluster_vec[4]->points.size()/2].x;
     KF4.statePre.at<float>(1)=cluster_vec[4]->points[cluster_vec[4]->points.size()/2].y;
     KF4.statePre.at<float>(2)=0;// initial v_x
     KF4.statePre.at<float>(3)=0;//initial v_y

     // Set initial state
     KF5.statePre.at<float>(0)=cluster_vec[5]->points[cluster_vec[5]->points.size()/2].x;
     KF5.statePre.at<float>(1)=cluster_vec[5]->points[cluster_vec[5]->points.size()/2].y;
     KF5.statePre.at<float>(2)=0;// initial v_x
     KF5.statePre.at<float>(3)=0;//initial v_y

     firstFrame=false;

     // Print the initial state of the kalman filter for debugging
     cout<<"KF0.satePre="<<KF0.statePre.at<float>(0)<<","<<KF0.statePre.at<float>(1)<<"\n";
     cout<<"KF1.satePre="<<KF1.statePre.at<float>(0)<<","<<KF1.statePre.at<float>(1)<<"\n";
     cout<<"KF2.satePre="<<KF2.statePre.at<float>(0)<<","<<KF2.statePre.at<float>(1)<<"\n";
     cout<<"KF3.satePre="<<KF3.statePre.at<float>(0)<<","<<KF3.statePre.at<float>(1)<<"\n";
     cout<<"KF4.satePre="<<KF4.statePre.at<float>(0)<<","<<KF4.statePre.at<float>(1)<<"\n";
     cout<<"KF5.satePre="<<KF5.statePre.at<float>(0)<<","<<KF5.statePre.at<float>(1)<<"\n";

     //cin.ignore();// To be able to see the printed initial state of the KalmanFilter



     /* Naive version without kalman filter */
     for (int i=0;i<6;i++)
     {
        geometry_msgs::Point pt;
        pt.x=cluster_vec[i]->points[cluster_vec[i]->points.size()/2].x;
        pt.y=cluster_vec[i]->points[cluster_vec[i]->points.size()/2].y;
        prevClusterCenters.push_back(pt);
     }

     /* Naive version without kalman filter */
}

 
else
{ 
  cout<<"ELSE firstFrame="<<firstFrame<<"\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      /* Creating the KdTree from input point cloud*/
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::fromROSMsg (*input, *input_cloud);

  tree->setInputCloud (input_cloud);

  /* Here we are creating a vector of PointIndices, which contains the actual index
   * information in a vector<int>. The indices of each detected cluster are saved here.
   * Cluster_indices is a vector containing one instance of PointIndices for each detected 
   * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
   */
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.08); 
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (600);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
cout<<"PCL init successfull\n";
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract (cluster_indices);
cout<<"PCL extract successfull\n";
  /* To separate each cluster out of the vector<PointIndices> we have to 
   * iterate through cluster_indices, create a new PointCloud for each 
   * entry and write all points of the current cluster in the PointCloud. 
   */
  //pcl::PointXYZ origin (0,0,0);
  //float mindist_this_cluster = 1000;
  //float dist_this_point = 1000;

  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  // Vector of cluster pointclouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vec;

  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for(pit = it->indices.begin(); pit != it->indices.end(); pit++) 
          {
          
                  cloud_cluster->points.push_back(input_cloud->points[*pit]);
                  //dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
                  //                                          origin);
                  //mindist_this_cluster = std::min(dist_this_point, mindist_this_cluster);
          }

         
      cluster_vec.push_back(cloud_cluster);

    }
    //cout<<"cluster_vec got some clusters\n";

    //Ensure at least 6 clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < 6){
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0,0,0));
      cluster_vec.push_back(empty_cluster);
    }
    std_msgs::Float32MultiArray cc;
    for(int i=0;i<6;i++)
    {
        cc.data.push_back(cluster_vec[i]->points[cluster_vec[i]->points.size()/2].x);
        cc.data.push_back(cluster_vec[i]->points[cluster_vec[i]->points.size()/2].y);
        cc.data.push_back(cluster_vec[i]->points[cluster_vec[i]->points.size()/2].z);

    }
    cout<<"6 clusters initialized\n";

    //cc_pos.publish(cc);// Publish cluster mid-points.
    KFT(cc);
    int i=0;
    bool publishedCluster[6];
    for(auto it=objID.begin();it!=objID.end();it++)
        { cout<<"Inside the for loop\n";
            
            switch(*it)
            {
                cout<<"Inside the switch case\n";
                case 0: { 
                            publish_cloud(pub_cluster0,cluster_vec[i]);
                            publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
                            i++;
                            break;

                        }
                case 1: {
                            publish_cloud(pub_cluster1,cluster_vec[i]);
                            publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
                            i++;
                            break;

                        }
                case 2: {
                            publish_cloud(pub_cluster2,cluster_vec[i]);
                            publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
                            i++;
                            break;

                        }
                case 3: {
                            publish_cloud(pub_cluster3,cluster_vec[i]);
                            publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
                            i++;
                            break;

                        }
                case 4: {
                            publish_cloud(pub_cluster4,cluster_vec[i]);
                            publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
                            i++;
                            break;

                        }

                case 5: {
                            publish_cloud(pub_cluster5,cluster_vec[i]);
                            publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
                            i++;
                            break;

                        }
                default: break;
            }
    
        }

} 

}   


  

int main(int argc, char** argv)
{
    // ROS init
    ros::init (argc,argv,"KFTracker");
    ros::NodeHandle nh;

   
    // Publishers to publish the state of the objects (pos and vel)
    //objState1=nh.advertise<geometry_msgs::Twist> ("obj_1",1);



cout<<"About to setup callback\n";

// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("filtered_cloud", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub_cluster0 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_0", 1);
  pub_cluster1 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_1", 1);
  pub_cluster2 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_2", 1);
  pub_cluster3 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_3", 1);
  pub_cluster4 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_4", 1);
  pub_cluster5 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_5", 1);
      // Subscribe to the clustered pointclouds
    //ros::Subscriber c1=nh.subscribe("ccs",100,KFT); 
    objID_pub = nh.advertise<std_msgs::Int32MultiArray>("obj_id", 1);
/* Point cloud clustering
*/
    
  //cc_pos=nh.advertise<std_msgs::Float32MultiArray>("ccs",100);//clusterCenter1


/* Point cloud clustering
*/    


    ros::spin();


}
