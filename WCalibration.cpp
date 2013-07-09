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
	 blobDetector = new cv::SimpleBlobDetector();

	 return true;
}

CALIB_SUCCESS Calibration::addImage(cv::Mat leftImage, cv::Mat rightImage, bool invert) {
	
	bool circleSuccess1, circleSuccess2;

	leftImage.copyTo(image1);
	rightImage.copyTo(image2);

	pointBuffer1.resize(boardSize.width*boardSize.height);
	pointBuffer2.resize(boardSize.width*boardSize.height);

	cv::SimpleBlobDetector::Params detectorParams;
	
	cv::Mat pointBufferTemp, pointBufferTemp2;

	//Camera 1
	cv::Mat gray;
	//cvtColor(image1Mat, gray, COLOR_BGR2GRAY);
	leftImage.copyTo(gray);


	if (invert) {gray = cvScalar(255) - gray;}

	circleSuccess1 = findCirclesGrid(gray, boardSize, pointBuffer1,  CALIB_CB_CLUSTERING | CALIB_CB_ASYMMETRIC_GRID, blobDetector);
	
	if (circleSuccess1) {
		imagePoints1.push_back(pointBuffer1);
		objectPoints1.push_back(objectPointsEach);
		currentFrame1++;
	}

	//Camera 2
	//cvtColor(image2Mat, gray, COLOR_BGR2GRAY);
	rightImage.copyTo(gray);
	if (invert) {gray = cvScalar(255) - gray;}
	circleSuccess2 = findCirclesGrid(gray, boardSize, pointBuffer2, CALIB_CB_ASYMMETRIC_GRID, blobDetector);

	if (circleSuccess2) {
		imagePoints2.push_back(pointBuffer2);
		objectPoints2.push_back(objectPointsEach);
		currentFrame2++;
	}

	//BOTH WORK!
	if (circleSuccess1 && circleSuccess2) {	
		imagePoints1All.push_back(pointBuffer1);
		imagePoints2All.push_back(pointBuffer2);
		objectPointsAll.push_back(objectPointsEach);
		return CALIB_SUCCESS_FRAME;
	}

	else if(circleSuccess1)
	{
		return CALIB_SUCCESS_1_ONLY;
	}
	else if(circleSuccess2)
	{
		return CALIB_SUCCESS_2_ONLY;
	}
}


CALIB_SUCCESS Calibration::processAll(void){
		Size imageSize(width,height);
		
		double rmsl = cv::calibrateCamera(objectPoints1, imagePoints1, imageSize, CameraParams1.cameraMatrix, CameraParams1.distCoeffs, rvecs, tvecs);
		double rmsr = cv::calibrateCamera(objectPoints2, imagePoints2, imageSize, CameraParams2.cameraMatrix, CameraParams2.distCoeffs, rvecs, tvecs);

		double rms = stereoCalibrate(objectPointsAll, imagePoints1All, imagePoints2All, CameraParams1.cameraMatrix, CameraParams1.distCoeffs, 
			CameraParams2.cameraMatrix, CameraParams2.distCoeffs, imageSize, R, T, E, F, TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
					CALIB_USE_INTRINSIC_GUESS + CV_CALIB_FIX_K1 + CV_CALIB_FIX_INTRINSIC);

		std::cout <<"Left"<<rmsl<<endl<<"Right"<<rmsr<<endl<< "Stereo Calibrate " << rms << std::endl;

		bool adjustFrac = true;
		float mFrac = -1;
		while (adjustFrac){
			
			stereoRectify(CameraParams1.cameraMatrix, CameraParams1.distCoeffs, CameraParams2.cameraMatrix, CameraParams2.distCoeffs, imageSize,
				R, T, CameraParams1.R, CameraParams2.R, CameraParams1.P, CameraParams2.P, CameraParams1.Q, 0, mFrac,imageSize);

			initUndistortRectifyMap(CameraParams1.cameraMatrix, CameraParams1.distCoeffs, CameraParams1.R, CameraParams1.P, imageSize, CV_32FC1, CameraParams1.rMapX, CameraParams1.rMapY);
			initUndistortRectifyMap(CameraParams2.cameraMatrix, CameraParams2.distCoeffs, CameraParams2.R, CameraParams2.P, imageSize, CV_32FC1, CameraParams2.rMapX, CameraParams2.rMapY);

		
			Mat undistortImgl;
			Mat undistorImagr;
			remap(image1, undistortImgl, CameraParams1.rMapX, CameraParams1.rMapY, INTER_LANCZOS4);
			remap(image2, undistorImagr, CameraParams2.rMapX, CameraParams2.rMapY, INTER_LANCZOS4);
			
			Mat canvas(height,width*2,CV_8UC1);
			for (int row = 0; row < height; row++) 
				for (int col = 0; col < width; col++) 
						canvas.data[row*width*2 + col] = undistortImgl.data[row*width+col];

			for (int row = 0; row < height; row++) 
				for (int col = 0; col < width; col++) 
						canvas.data[width + row*width*2 + col] = undistorImagr.data[row*width+col];

			
			namedWindow("UndistortL", WINDOW_AUTOSIZE);
			imshow("UndistortL", canvas);
			cv::waitKey(0);
			destroyAllWindows();
		
			
			std::string shouldAdjustFrac = "blah";
			
			while(!(shouldAdjustFrac == "y" || shouldAdjustFrac == "n"))
			{
				std::cout << "Should I adjust fraction? (Y/n) :";
				std::cin >> shouldAdjustFrac;
				if (shouldAdjustFrac == "y"){
					std::cout << "new fraction: ";
					std::cin >> mFrac;
					adjustFrac = true;
				} else {
					adjustFrac = false;
				}
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
		
		// save calibration parms to output
		cv::FileStorage fs("output.yaml", cv::FileStorage::WRITE);
		fs << "Q" << CameraParams1.Q;
		fs << "C1" << CameraParams1.cameraMatrix;
		fs << "D1" << CameraParams1.distCoeffs;
		fs << "PRinv1" << CameraParams1.PRinv;
		fs << "P1" << CameraParams1.P;
		fs << "R1" << CameraParams1.R;
 
		fs << "C2" << CameraParams2.cameraMatrix;
		fs << "D2" << CameraParams2.distCoeffs;
		fs << "PRinv2" << CameraParams2.PRinv;
		fs << "P2" << CameraParams2.P;
		fs << "R2" << CameraParams2.R;
				
		fs.release();


		return CALIB_SUCCESS_TOTAL;
}

vector<vector<Mat>> Calibration::RectifyImages(vector<Mat> leftImages, vector<Mat> rightImages)
{
	//read in the config 

	int numberOfImages = leftImages.size();
	vector<Mat> LeftRectifiedImages(numberOfImages);
	vector<Mat> RightRectifiedImages(numberOfImages);
	vector<Mat> StichtedImages(numberOfImages);

	vector<vector<Mat>> RectifiedImages(3);
	RectifiedImages[0] = LeftRectifiedImages;
	RectifiedImages[1] = RightRectifiedImages;
	RectifiedImages[2] = StichtedImages;

	cv::FileStorage fs("output.yaml", cv::FileStorage::READ);

	fs["Q"] >> CameraParams1.Q;
	fs ["C1"] >>CameraParams1.cameraMatrix;
	fs ["D1"] >>CameraParams1.distCoeffs;
	fs ["PRinv1"] >> CameraParams1.PRinv;
	fs ["P1"] >>CameraParams1.P;
	fs ["R1"]>> CameraParams1.R;
 
	fs ["C2"] >>CameraParams2.cameraMatrix;
	fs  ["D2"]>>CameraParams2.distCoeffs;
	fs  ["PRinv2"] >>CameraParams2.PRinv;
	fs ["P2"] >>CameraParams2.P;
	fs ["R2"] >> CameraParams2.R;	
	fs.release();

	cv::Size imageSize(width,height);
	initUndistortRectifyMap(CameraParams1.cameraMatrix, CameraParams1.distCoeffs, CameraParams1.R, CameraParams1.P, imageSize, CV_32FC1, CameraParams1.rMapX, CameraParams1.rMapY);
	initUndistortRectifyMap(CameraParams2.cameraMatrix, CameraParams2.distCoeffs, CameraParams2.R, CameraParams2.P, imageSize, CV_32FC1, CameraParams2.rMapX, CameraParams2.rMapY);


	for( int i =0;i< numberOfImages;i++)
	{
		Mat undistortImgl;
		Mat undistorImagr;

		remap(leftImages[i], undistortImgl, CameraParams1.rMapX, CameraParams1.rMapY, INTER_LANCZOS4);
		remap(rightImages[i], undistorImagr, CameraParams2.rMapX, CameraParams2.rMapY, INTER_LANCZOS4);
		RectifiedImages[0][i] = undistortImgl;
		RectifiedImages[1][i] = undistortImgl;

		Mat canvas(height,width*2,CV_8UC1);
		for (int row = 0; row < height; row++) 
			for (int col = 0; col < width; col++) 
					canvas.data[row*width*2 + col] = undistortImgl.data[row*width+col];

		for (int row = 0; row < height; row++) 
			for (int col = 0; col < width; col++) 
					canvas.data[width + row*width*2 + col] = undistorImagr.data[row*width+col];


		RectifiedImages[2][i] = canvas;



	}

	

	return RectifiedImages;
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


