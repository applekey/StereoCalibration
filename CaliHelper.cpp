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

unique_ptr<Mat[]> CaliHelper::GetCalibrationImages(int numImag)
{
    char key = 'c';
	int capturedImages = 0;

	namedWindow( "Camera", CV_WINDOW_NORMAL  );

	PGCam pgcam;
	pgcam.Init(CAMERA_WIDTH,CAMERA_HEIGHT,CAMERA_OFFSET_X,CAMERA_OFFSET_Y);

	unique_ptr<Mat[]> caliImages(new Mat[numImag*2]);
	while(key!='q')
	{
		vector<IplImage*> image = pgcam.grabFrame();
		
		char key = cvWaitKey(1);
		if(key == 'c')
		{
			if(capturedImages/2 == numImag)
				break;

			Mat(image[0]).copyTo(caliImages[capturedImages]);
			capturedImages+=1;
			Mat(image[1]).copyTo(caliImages[capturedImages]);
			capturedImages+=1;


			IplImage* view2=cvCloneImage(&(IplImage)Mat(image[0]));
			cvNot(view2,view2);
			imshow( "Camera", Mat(view2) );
			cvReleaseImage(&view2);
		}
		else
		{
			cv::Mat im1, im2;
			cv::resize(Mat(image[0]), im1, cv::Size(800,400));
			cv::resize(Mat(image[1]), im2, cv::Size(800,400));
			//imshow( "Camera", image[0] );
			imshow("left", im1);
			imshow("right", im2);
			cvReleaseImage(&image[0]);
			cvReleaseImage(&image[1]);
		}
	}
	cvDestroyWindow("Camera");
  
	return caliImages;
}	


void CaliHelper::SaveImages(Mat* images, int frames,string directory)
{
	for(int i=0;i<frames;i++)
	{
		char fullDir[100];
		sprintf(fullDir,"\\Image%d.png",i+1);
		string fulldir = directory + fullDir;
		imwrite( fulldir,images[i] );
	}	
}

void CaliHelper::WriteParmsToFile(CameraParams cameraL,CameraParams cameraR)
{
	//WRITE YAML FILE
		cv::FileStorage fs("output.yaml", cv::FileStorage::WRITE);
		fs << "Q" << cameraL.Q;
		fs << "C1" << cameraL.cameraMatrix;
		fs << "D1" << cameraL.distCoeffs;
		fs << "PRinv1" << cameraL.PRinv;
		fs << "P1" << cameraL.P;
		fs << "R1" << cameraL.R;
 
		fs << "C2" << cameraR.cameraMatrix;
		fs << "D2" << cameraR.distCoeffs;
		fs << "PRinv2" << cameraR.PRinv;
		fs << "P2" << cameraR.P;
		fs << "R2" << cameraR.R;
					
		fs.release();
}

bool CaliHelper::RectifyImages(Mat* CameraImages, int numberOfImages,string outputLocation)
{
	CameraParams cameraL;
	CameraParams cameraR;

	cv::FileStorage fs("output.yaml", cv::FileStorage::READ);

	fs["Q"] >> cameraL.Q;
	fs ["C1"] >>cameraL.cameraMatrix;
	fs ["D1"] >>cameraL.distCoeffs;
	fs ["PRinv1"] >> cameraL.PRinv;
	fs ["P1"] >>cameraL.P;
	fs ["R1"]>> cameraL.R;
 
	fs ["C2"] >>cameraR.cameraMatrix;
	fs  ["D2"]>>cameraR.distCoeffs;
	fs  ["PRinv2"] >>cameraR.PRinv;
	fs ["P2"] >>cameraR.P;
	fs ["R2"] >> cameraR.R;
					
	fs.release();

	// DO SOME RECTIFICATION

	cvNamedWindow("bal");
	for(int i=0;i<numberOfImages*2;i++)
	{
	  imshow("bal",CameraImages[i]);
	  cvWaitKey(0);
	}

	cv::Mat imageIn, imageRectified;
	cv::Mat mapX, mapY;
	cv::Mat output;
	
	std::vector<CameraParams> mParams;
	mParams.emplace_back(cameraL);
	mParams.emplace_back(cameraR);
 
	int width = 800, height = 600;
	cv::Mat canvas(height, width*2, CV_8UC3);
	
	//number of cameras
	for (int j = 0; j < 2; j++){
		CameraParams x = mParams[j];
		initUndistortRectifyMap(x.cameraMatrix, x.distCoeffs, x.R, x.P, cv::Size(width, height), CV_32FC1, mapX, mapY);
 
		cout << "width,height = " << width << "," << height << endl;
		cout << "C1 " << x.cameraMatrix << endl;
		cout << "D1 " << x.distCoeffs << endl;
		cout << "P1 " << x.P << endl;
 
		//number of images
		for (int i = 0; i <numberOfImages; i++){

			remap(CameraImages[i], imageRectified, mapX, mapY, CV_INTER_LANCZOS4);
			//sprintf(outputLocation.c_str(), "rectified/%d_%04d.png",j, i);
			//cv::imwrite(filename, imageRectified);
			//if (i == 9 && j == 0){
			//	for (int row = 0; row < height; row++) for (int col = 0; col < width; col++) for (int chan = 0; chan < 3; chan++) canvas.data[row*width*2*3 + col*3 + chan] = imageRectified.data[row*width*3+col*3 + chan];
			//} else if (i == 9 && j == 1){
			//	for (int row = 0; row < height; row++) for (int col = 0; col < width; col++) for (int chan = 0; chan < 3; chan++) canvas.data[width*3 + row*width*2*3 + col*3 + chan] = imageRectified.data[row*width*3+col*3 + chan];
			//}
		} 
	}



	return false;

}
