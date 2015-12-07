#include <iostream>
#include "CKalmanFilter.h"

using namespace cv;
using namespace std;

// Constructor
CKalmanFilter::CKalmanFilter(vector<Vec2f> p){

	kalman = new KalmanFilter( 4, 4, 0 ); // 4 measurement and state parameters
	kalman->transitionMatrix = (Mat_<float>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);

	// Initialization
	prevResult = p;
	kalman->statePre.at<float>(0) = p[0][0]; // r1
	kalman->statePre.at<float>(1) = p[0][1]; // theta1
	kalman->statePre.at<float>(2) = p[1][0]; // r2
	kalman->statePre.at<float>(3) = p[1][1]; // theta2

	kalman->statePost.at<float>(0)=p[0][0];
	kalman->statePost.at<float>(1)=p[0][1];
	kalman->statePost.at<float>(2)=p[1][0];
	kalman->statePost.at<float>(3)=p[1][1];

	setIdentity(kalman->measurementMatrix);
	setIdentity(kalman->processNoiseCov, Scalar::all(1e-4));
	setIdentity(kalman->measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(kalman->errorCovPost, Scalar::all(5));
}

// Destructor
CKalmanFilter::~CKalmanFilter(){
	delete kalman;
}

// Prediction
vector<Vec2f> CKalmanFilter::predict(){
	Mat prediction = kalman->predict(); // predict the state of the next frame
	prevResult[0][0] = prediction.at<float>(0);prevResult[0][1] = prediction.at<float>(1);
	prevResult[1][0] = prediction.at<float>(2);prevResult[1][1] = prediction.at<float>(3);
	return prevResult;

}

// Correct the prediction based on the measurement
vector<Vec2f> CKalmanFilter::update(vector<Vec2f> measure){

	
	Mat_<float> measurement(4,1);
	measurement.setTo(Scalar(0));

	measurement.at<float>(0) = measure[0][0];measurement.at<float>(1) = measure[0][1];
	measurement.at<float>(2) = measure[1][0];measurement.at<float>(3) = measure[1][1];

	Mat estimated = kalman->correct(measurement); // Correct the state of the next frame after obtaining the measurements
	
	// Update the measurement	
	if(estimated.at<float>(0) < estimated.at<float>(2)){
		measure[0][0] = estimated.at<float>(0);measure[0][1] = estimated.at<float>(1);
		measure[1][0] = estimated.at<float>(2);measure[1][1] = estimated.at<float>(3);
	}
	else{
		measure[0][0] = estimated.at<float>(2);measure[0][1] = estimated.at<float>(3);
		measure[1][0] = estimated.at<float>(0);measure[1][1] = estimated.at<float>(1);
	}

	waitKey(1);
	
	return measure; // return the measurement

}