#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>



#define SAVE_DIRECTORY_LEFT "C:\\Users\\linda\\Desktop\\LeftCamera"
#define NUMBER_OF_IMAGES 15

using namespace cv;
using namespace std;

#define ROW_SIZE 4
#define WIDTH_SIZE 11
#define BLOCK_SIZE 10
#define FIX_ASPECT_RATIO 1

//#define DEBUG 1





bool runCalibrationAndSave( Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );



void LoadImageList(Mat* images,string directory, int number)
{
	char imageName[100] ;

	for( int i =0;i<number;i++)
	{
 		sprintf(imageName, "\\Image%d.png",i+1);
		string imageFullPath = directory + string(imageName);

		images[i]= imread(imageFullPath,0);
	
	}

}



int main(int argc, char* argv[])
{
	//Load Images
	Mat leftImages[NUMBER_OF_IMAGES];
	
	LoadImageList(leftImages,SAVE_DIRECTORY_LEFT,NUMBER_OF_IMAGES);


	// DO IT
    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    cv::Size imageSize = leftImages[0].size();

    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

	Mat foundCircles[NUMBER_OF_IMAGES];

    for(int i = 0;i<NUMBER_OF_IMAGES;++i)
    {
      Mat view;
      bool blinkOutput = false;

      view = leftImages[i];

	  //////////////////
	  // find black dots, gotta invert the colors
	  IplImage* view2=cvCloneImage(&(IplImage)view);
	  cvNot(view2,view2);


		vector<Point2f> pointBuf;
		cv::Size boardSize(ROW_SIZE,WIDTH_SIZE);

        bool found;
        found = findCirclesGrid( Mat(view2),boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID  );

        if ( found)                // If done with success,
        {
		   imagePoints.push_back(pointBuf);
           drawChessboardCorners( view, boardSize, Mat(pointBuf), found );
        }
		foundCircles[i] = view;

		cout<<"Processing frame.\n";
    }

#ifdef DEBUG
	//Display the frames
	cv::namedWindow( "Found Circles", CV_WINDOW_NORMAL);
	for(int i=0;i<NUMBER_OF_IMAGES;i++)
	{
		imshow("D",foundCircles[i]);
		cvWaitKey(0);
	}
#endif

	// NOW CALIBRATE

	 if(!runCalibrationAndSave(imageSize,  cameraMatrix, distCoeffs, imagePoints))
	 {
		return -1;
	 }

	//SHOW UNDISTORTED

	Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
            imageSize, CV_16SC2, map1, map2);

	for(int i = 0; i < NUMBER_OF_IMAGES; i++ )
	{
		view = leftImages[i];
	
		remap(view, rview, map1, map2, INTER_LINEAR);
		imshow("Image View", rview);
		waitKey(0);
	}
    return 0;
}

static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
    corners.clear();

    for( int i = 0; i < boardSize.height; i++ )
	{
		for( int j = 0; j < boardSize.width; j++ )
            corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
	}
        
       
    
}


static bool runCalibration( Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{

   cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( FIX_ASPECT_RATIO & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);

	Size boardSize(ROW_SIZE,WIDTH_SIZE);
	Size blockSize(BLOCK_SIZE,BLOCK_SIZE);

    calcBoardCornerPositions(boardSize, BLOCK_SIZE, objectPoints[0]);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, FIX_ASPECT_RATIO|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

	 cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);


    return true;
}


bool runCalibrationAndSave(Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed");

    return ok;
}
