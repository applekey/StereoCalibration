#include "Calibration.h"

Calibration::Calibration(Size patternSize,int blockSize,int fixAspect)
{
	IsCalibrated = false;
	caliParms.blockSize = blockSize;
	caliParms.fixAspect = fixAspect;
	caliParms.patternSize = patternSize;
}

int Calibration::Calibrate(Mat* images,int numberOfImages)
{
	vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    cv::Size imageSize = images[0].size();

  
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

	unique_ptr<Mat[]> foundCircles (new Mat[numberOfImages]);

    for(int i = 0;i<numberOfImages;++i)
    {
      Mat view;
      bool blinkOutput = false;

      view = images[i];

	  //////////////////
	  // find black dots, gotta invert the colors
	  IplImage* view2=cvCloneImage(&(IplImage)view);
	  cvNot(view2,view2);


		vector<Point2f> pointBuf;
		cv::Size boardSize(caliParms.patternSize.width,caliParms.patternSize.height);

        bool found;
        found = findCirclesGrid( Mat(view2),boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID  );

        if ( !found)                // If done with success,
        {
		   cout<< "Cannot find dots for one image.\n";
		   continue;
        }
		imagePoints.push_back(pointBuf);
        drawChessboardCorners( view, boardSize, Mat(pointBuf), found );

		foundCircles[i] = view;

		cout<<"Processing frame.\n";
    }

#ifdef DEBUG
	//Display the frames
	cv::namedWindow( "Found Circles", CV_WINDOW_NORMAL);
	for(int i=0;i<numberOfImages;i++)
	{
		imshow("D",foundCircles[i]);
		cvWaitKey(0);
	}
#endif

	// NOW CALIBRATE
	 cout << "Finding int and ext parms.\n";
	 if(!runCalibrationAndSave(imageSize,  cameraMatrix, distCoeffs, imagePoints))
	 {
		return -1;
	 }
	 cameraMatrix.copyTo(caliParms.cameraMatrix);
	 distCoeffs.copyTo(caliParms.distCoeffs);
	 
	 IsCalibrated = true;
	 cout<<"Calibration Succeeded.\n";
	 return 0;

}

Mat Calibration::RectifyImage(Mat image)
{
	Size imgSize = image.size();
	Mat view, rview, map1, map2;
    initUndistortRectifyMap(caliParms.cameraMatrix, caliParms.distCoeffs, Mat(),
        getOptimalNewCameraMatrix(caliParms.cameraMatrix, caliParms.distCoeffs, imgSize, 1, imgSize, 0),
        imgSize, CV_16SC2, map1, map2);

	remap(image, rview, map1, map2, INTER_LINEAR);

    return rview;
}



double Calibration::computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
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

void Calibration::calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
    corners.clear();

    for( int i = 0; i < boardSize.height; i++ )
	{
		for( int j = 0; j < boardSize.width; j++ )
            corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
	}
        
       
    
}


bool Calibration::runCalibration( Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{

   cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( caliParms.fixAspect & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);

	Size boardSize(caliParms.patternSize.height,caliParms.patternSize.width);

	calcBoardCornerPositions(boardSize, caliParms.blockSize, objectPoints[0]);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, caliParms.fixAspect|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

	 cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return true;
}


bool Calibration::runCalibrationAndSave(Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed");

    return ok;
}

Mat Calibration::RectifyImageAlt(Mat image, Mat cameraMatrix, Mat distCoeffs)
{
	Size imgSize = image.size();
	Mat view, rview, map1, map2;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imgSize, 1, imgSize, 0),
        imgSize, CV_16SC2, map1, map2);

	remap(image, rview, map1, map2, INTER_LINEAR);

    return rview;
}




