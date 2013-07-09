#include "CaliHelper.h"



unique_ptr<Mat[]> CaliHelper::GrabFringeImages( string* images,int nFrames)
{
	PGCam pgcam;
	pgcam.Init(CAMERA_WIDTH,CAMERA_HEIGHT,CAMERA_OFFSET_X,CAMERA_OFFSET_Y);
	LightCrafter lcr;

	unique_ptr<Mat[]> fringeFrames ( new Mat[nFrames*2]);

	for( int i =0;i<nFrames;i++)
	{
		Mat image = imread(images[i]);
		lcr.ProjectImage(image);
		Sleep(1000);
		fringeFrames[i*2] = pgcam.grabFrame()[0];
		fringeFrames[i*2+1] = pgcam.grabFrame()[1];
			
	}
	lcr.Disconnect();

	return fringeFrames;
}

vector<unique_ptr<Mat[]>> CaliHelper::GetCalibrationImages(int numImag)
{
	// output
	unique_ptr<Mat[]> LeftImages(new Mat[numImag]);
	unique_ptr<Mat[]> RightImages(new Mat[numImag]);
	vector<unique_ptr<Mat[]>> calibrationImages(2);
	

	// Setup Calibration
	Calibration calibration(CALIBRATION_BOARD_WIDTH,CALIBRATION_BOARD_HEIGHT,DISTANCE_TO_CENTER_MM);
	int successfulRightFrames= 0;
	int successfulLeftFrames = 0;

	// Project a white image to help with calibration;
	LightCrafter lcr;
	Mat whiteImage(684,608, CV_8UC1, Scalar(255));
	lcr.ProjectImage(whiteImage);

    char key = 'c';
	int capturedImages = 0;

	namedWindow( "left");
	namedWindow( "right");

	PGCam pgcam;
	pgcam.Init(CAMERA_WIDTH,CAMERA_HEIGHT,CAMERA_OFFSET_X,CAMERA_OFFSET_Y);

	while(key!='q')
	{
		vector<IplImage*> image = pgcam.grabFrame();
		
		char key = cvWaitKey(1);
		if(key == 'c')
		{
			if(successfulRightFrames== numImag && successfulLeftFrames == numImag)
				break;

			Mat leftImage(image[0]);
			Mat rightImage(image[1]);

			CALIB_SUCCESS success = calibration.addImage(leftImage,rightImage,true);
			if(success == CALIB_SUCCESS_1_ONLY)
			{
				LeftImages[successfulLeftFrames] = leftImage;
				successfulLeftFrames++;
				
			}
			if(CALIB_SUCCESS_2_ONLY)
			{
				RightImages[successfulRightFrames] = rightImage;
				successfulRightFrames++;
			}
			if(CALIB_SUCCESS_FRAME)
			{
				LeftImages[successfulLeftFrames] = leftImage;
				RightImages[successfulRightFrames] = rightImage;
				successfulRightFrames++;
				successfulLeftFrames++;
			}


			IplImage* view2=cvCloneImage(&(IplImage)Mat(image[0]));
			cvNot(view2,view2);
			imshow( "left", Mat(view2) );
			// DO CALIBRATION

			cvReleaseImage(&view2);
		}
		else
		{
			cv::Mat im1, im2;
			cv::resize(Mat(image[0]), im1, cv::Size(800,600));
			cv::resize(Mat(image[1]), im2, cv::Size(800,600));
			//imshow( "Camera", image[0] );
			imshow("left", im1);
			imshow("right", im2);
			cvReleaseImage(&image[0]);
			cvReleaseImage(&image[1]);
		}
	}

	calibrationImages[0].swap(LeftImages);
	calibrationImages[1].swap(RightImages);

	cvDestroyWindow("Camera");
  
	return calibrationImages;
}	

Mat CaliHelper::RotateImage(const Mat& source, double angle)
{

	 Point2f src_center(source.cols/2.0F, source.rows/2.0F);
    Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
    Mat dst;
    warpAffine(source, dst, rot_mat, source.size());
    return dst;
}
