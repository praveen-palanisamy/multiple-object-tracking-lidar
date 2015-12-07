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


    cv::Mat state(stateDim,1,CV_32F);
    cv::Mat_<float> measurement(2,1); 
   // measurement.setTo(Scalar(0));


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
void KFT(const std_msgs::Float32MultiArray::ConstPtr& ccs)
{



    // First predict, to update the internal statePre variable
   /* Mat prediction0 = KF0.predict();
    Mat prediction1 = KF1.predict();
    Mat prediction2 = KF2.predict();
    Mat prediction3 = KF3.predict();
    Mat prediction4 = KF4.predict();
    Mat prediction5 = KF5.predict();
    */

    std::vector<cv::Mat> pred{KF0.predict(),KF1.predict(),KF2.predict(),KF3.predict(),KF4.predict(),KF5.predict()};
 //cout<<"Pred successfull\n";

    //cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
   // cout<<"Prediction 1 ="<<prediction.at<float>(0)<<","<<prediction.at<float>(1)<<"\n";

    // Get measurements
    // Extract the position of the clusters forom the multiArray. To check if the data
    // coming in, check the .z (every third) coordinate and that will be 0.0
    std::vector<geometry_msgs::Point> clusterCenters;//clusterCenters
   
    int i=0;
    for (std::vector<float>::const_iterator it=ccs->data.begin();it!=ccs->data.end();it+=3)
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

    std::vector<int> objID;
    
    // Find the cluster that is more probable to be belonging to a given KF.
   
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

    //display objIDs
  /* DEBUG
    cout<<"objID= ";
    for(auto it=objID.begin();it!=objID.end();it++)
        cout<<*it<<" ,";
    cout<<"\n";
    */
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
 
    

   // cout<<"estimate="<<estimated.at<float>(0)<<","<<estimated.at<float>(1)<<"\n";
   // Point statePt(estimated.at<float>(0),estimated.at<float>(1));
//cout<<"DONE KF_TRACKER\n";
   
}


int main(int argc, char** argv)
{
    // ROS init
    ros::init (argc,argv,"KFTracker");
    ros::NodeHandle nh;

   
    // Publishers to publish the state of the objects (pos and vel)
    //objState1=nh.advertise<geometry_msgs::Twist> ("obj_1",1);

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

     // Set initial state
     KF0.statePre.at<float>(0)=0.0;//initial x pos of the cluster
     KF0.statePre.at<float>(1)=0.0;//initial y pos of the cluster
     KF0.statePre.at<float>(2)=0;// initial v_x
     KF0.statePre.at<float>(3)=0;//initial v_y

     // Set initial state
     KF1.statePre.at<float>(0)=0.0;//initial x pos of the cluster
     KF1.statePre.at<float>(1)=0.0;//initial y pos of the cluster
     KF1.statePre.at<float>(2)=0;// initial v_x
     KF1.statePre.at<float>(3)=0;//initial v_y

     // Set initial state
     KF2.statePre.at<float>(0)=0.0;//initial x pos of the cluster
     KF2.statePre.at<float>(1)=0.0;//initial y pos of the cluster
     KF2.statePre.at<float>(2)=0;// initial v_x
     KF2.statePre.at<float>(3)=0;//initial v_y


     // Set initial state
     KF3.statePre.at<float>(0)=0.0;//initial x pos of the cluster
     KF3.statePre.at<float>(1)=0.0;//initial y pos of the cluster
     KF3.statePre.at<float>(2)=0;// initial v_x
     KF3.statePre.at<float>(3)=0;//initial v_y

     // Set initial state
     KF4.statePre.at<float>(0)=0.0;//initial x pos of the cluster
     KF4.statePre.at<float>(1)=0.0;//initial y pos of the cluster
     KF4.statePre.at<float>(2)=0;// initial v_x
     KF4.statePre.at<float>(3)=0;//initial v_y

     // Set initial state
     KF5.statePre.at<float>(0)=0.0;//initial x pos of the cluster
     KF5.statePre.at<float>(1)=0.0;//initial y pos of the cluster
     KF5.statePre.at<float>(2)=0;// initial v_x
     KF5.statePre.at<float>(3)=0;//initial v_y

cout<<"About to setup callback";
      // Subscribe to the clustered pointclouds
    ros::Subscriber c1=nh.subscribe("ccs",100,KFT); 
    objID_pub = nh.advertise<std_msgs::Int32MultiArray>("obj_id", 1);

    ros::spin();


}
