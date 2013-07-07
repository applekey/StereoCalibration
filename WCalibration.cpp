#include "WCalibration.h"
#include <iostream>


using namespace std;
using namespace cv;

Calibration::Calibration(int mWidth, int mHeight, float mDistCenterToCenterInMM) : 
								width(mWidth), height(mHeight), distCenterToCenterInMM(mDistCenterToCenterInMM){
					
	init(width, height, distCenterToCenterInMM);
}

bool Calibration::init(int mWidth, int mHeight, float mDistCenterToCenterInMM){
	 width = mWidth;
	 height = mHeight;
	 distCenterToCenterInMM = mDistCenterToCenterInMM;
	 boardSize.width = 4;
	 boardSize.height = 11;
	 objectPointsEach = calcObjectPoints();
	 currentFrame = 0;
	 blobDetector = new cv::SimpleBlobDetector();

	 return true;
}

CALIB_SUCCESS Calibration::addImage(cv::Mat image1Mat, cv::Mat image2Mat, bool invert) {
	
	image1Mat.copyTo(image1);
	image2Mat.copyTo(image2);
	
	bool circleSuccess1, circleSuccess2;

	pointBuffer1.resize(boardSize.width*boardSize.height);
	pointBuffer2.resize(boardSize.width*boardSize.height);

	cv::SimpleBlobDetector::Params detectorParams;
	
	cv::Mat pointBufferTemp, pointBufferTemp2;

	//Camera 1
	cv::Mat gray;
	//cvtColor(image1Mat, gray, COLOR_BGR2GRAY);
	image1Mat.copyTo(gray);
	if (invert) {gray = cvScalar(255) - gray;}

	circleSuccess1 = findCirclesGrid(gray, boardSize, pointBuffer1,  CALIB_CB_CLUSTERING | CALIB_CB_ASYMMETRIC_GRID, blobDetector);
	
	if (circleSuccess1) {
		imagePoints1.push_back(pointBuffer1);
		objectPoints1.push_back(objectPointsEach);
		currentFrame1++;
	}

	//Camera 2
	//cvtColor(image2Mat, gray, COLOR_BGR2GRAY);
	image2Mat.copyTo(gray);
	if (invert) {gray = cvScalar(255) - gray;}
	circleSuccess2 = findCirclesGrid(gray, boardSize, pointBuffer2, CALIB_CB_ASYMMETRIC_GRID, blobDetector);

	if (circleSuccess2) {
		imagePoints2.push_back(pointBuffer2);
		objectPoints2.push_back(objectPointsEach);
		currentFrame2++;
	}

	//BOTH WORK!
	if (circleSuccess1 && circleSuccess2) {
		cout << currentFrame << endl;
		imagePoints1.push_back(pointBuffer1);
		imagePoints2.push_back(pointBuffer2);
		objectPointsAll.push_back(objectPointsEach);
	}


	if (circleSuccess1 && circleSuccess2 && currentFrame == maxFrames - 1){
		
	} else if (circleSuccess1 && circleSuccess2) {
		currentFrame++;
		return CALIB_SUCCESS_FRAME;
	} else {
		return CALIB_FAILURE;
	}
	
}


CALIB_SUCCESS Calibration::processAll(void){
		cv::calibrateCamera(objectPoints1, imagePoints1, cv::Size(width, height), CameraParams1.cameraMatrix, CameraParams1.distCoeffs, rvecs, tvecs);
		cv::calibrateCamera(objectPoints2, imagePoints2, cv::Size(width, height), CameraParams2.cameraMatrix, CameraParams2.distCoeffs, rvecs, tvecs);

		double rms = stereoCalibrate(objectPointsAll, imagePoints1All, imagePoints2All, CameraParams1.cameraMatrix, CameraParams1.distCoeffs, 
			CameraParams2.cameraMatrix, CameraParams2.distCoeffs, cv::Size(width, height), R, T, E, F, TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
					CALIB_USE_INTRINSIC_GUESS + CV_CALIB_FIX_K1 + CV_CALIB_FIX_INTRINSIC);

		std::cout << "Stereo Calibrate " << rms << std::endl;

		bool adjustFrac = true;
		float mFrac = 1;
		while (adjustFrac){

			stereoRectify(CameraParams1.cameraMatrix, CameraParams1.distCoeffs, CameraParams2.cameraMatrix, CameraParams2.distCoeffs, cv::Size(width, height),
				R, T, CameraParams1.R, CameraParams2.R, CameraParams1.P, CameraParams2.P, CameraParams1.Q, 0, mFrac, cv::Size(width, height), &CameraParams1.validRoi, &CameraParams2.validRoi);

			initUndistortRectifyMap(CameraParams1.cameraMatrix, CameraParams1.distCoeffs, CameraParams1.R, CameraParams1.P, cv::Size(width,height), CV_32FC1, CameraParams1.rMapX, CameraParams1.rMapY);
			initUndistortRectifyMap(CameraParams2.cameraMatrix, CameraParams2.distCoeffs, CameraParams2.R, CameraParams2.P, cv::Size(width,height), CV_32FC1, CameraParams2.rMapX, CameraParams2.rMapY);

			canvas = Mat(height, width*2, CV_8UC3);
			Mat undistortImg;
			//remap(image1, undistortImg, CameraParams1.rMapX, CameraParams1.rMapY, INTER_LANCZOS4);
			//for (int row = 0; row < height; row++) for (int col = 0; col < width; col++) for (int chan = 0; chan < 3; chan++) canvas.data[row*width*2*3 + col*3 + chan] = undistortImg.data[row*width*3+col*3 + chan];

			//remap(image2, undistortImg, CameraParams2.rMapX, CameraParams2.rMapY, INTER_LANCZOS4);
			//for (int row = 0; row < height; row++) for (int col = 0; col < width; col++) for (int chan = 0; chan < 3; chan++) canvas.data[width*3 + row*width*2*3 + col*3 + chan] = undistortImg.data[row*width*3+col*3 + chan];
		
			//namedWindow("Undistort", WINDOW_AUTOSIZE);
			//imshow("Undistort", canvas);
			cv::waitKey(30);
			std::string shouldAdjustFrac;
			std::cout << "Should I adjust fraction? (Y/n) :";
			std::cin >> shouldAdjustFrac;
			if (shouldAdjustFrac == "Y" || shouldAdjustFrac == "y"){
				std::cout << "new fraction: ";
				std::cin >> mFrac;
				adjustFrac = true;
			} else {
				adjustFrac = false;
			}

		}

		CameraParams1.PRinv = (CameraParams1.P.clone().colRange(0,3)*CameraParams1.R.clone()).inv(DECOMP_LU);
		CameraParams2.PRinv = (CameraParams2.P.clone().colRange(0,3)*CameraParams2.R.clone()).inv(DECOMP_LU);

		cout << "width,height = " << width << "," << height << endl;
		cout << "C1 " << CameraParams1.cameraMatrix << endl;
		cout << "D1 " << CameraParams1.distCoeffs << endl;
		cout << "P1 " << CameraParams1.P << endl;
		cout << "C2 " << CameraParams2.cameraMatrix << endl;
		cout << "D2 " << CameraParams2.distCoeffs << endl;
		cout << "P2 " << CameraParams2.P << endl;
		

		currentFrame = 0;
		return CALIB_SUCCESS_TOTAL;
}

std::vector<cv::Point3f> Calibration::calcObjectPoints(){
	vector<Point3f> mObjectPoints;
	//cv::Mat mObjectPoints;
	for( int i = 0; i < boardSize.height; i++ ){
		for( int j = 0; j < boardSize.width; j++ ){
			mObjectPoints.push_back(Point3f(float((2*j + i % 2)*distCenterToCenterInMM), float(i*distCenterToCenterInMM), 0));
		}
	}
	
	return mObjectPoints;
};
