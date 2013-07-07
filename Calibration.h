#pragma once

#include <memory>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>



using namespace cv;
using namespace std;

class Calibration
{
private:

	bool IsCalibrated;

	
	
	Calibration();
	
	double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
											 const vector<vector<Point2f> >& imagePoints,
											 const vector<Mat>& rvecs, const vector<Mat>& tvecs,
											 const Mat& cameraMatrix , const Mat& distCoeffs,
											 vector<float>& perViewErrors);

	void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners);

	bool runCalibration( Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr);

	bool runCalibrationAndSave(Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints );


public:

	typedef struct {
		cv::Mat cameraMatrix;
		cv::Mat distCoeffs;
		cv::Rect validRoi;
		cv::Mat Q;
		cv::Mat R;
		cv::Mat P;
		cv::Mat PRinv;
	} CameraParams;


	typedef struct CalibrationParamters
	{
		cv::Size patternSize;
		int blockSize;
		int fixAspect;
		int numberOfImages;
		Mat cameraMatrix;
		Mat distCoeffs;
	} CaliParms;

	CaliParms caliParms;

	Calibration(cv::Size patternSize,int blockSize,int fixAspect);
	
	int Calibrate(Mat* images,int numberOfImages);

	Mat RectifyImage(Mat image);
	static Mat RectifyImageAlt(Mat image, Mat cameraMatrix, Mat distCoeffs); 

};

