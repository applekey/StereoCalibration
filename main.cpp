#include <stdio.h>

#include <string>
#include <iostream>
#include "CaliHelper.h"

#include <math.h>


using namespace std;

//-----------------------------SET PARMS------------------------------------------------------////


#define NUMBER_OF_CALIBRATION_IMAGES 10

#define NUMBER_OF_CAPTURE_IMAGES 3


#define FRINGE_IMAGE_LOCATION1 "CapturePatterns//0001.bmp"
#define FRINGE_IMAGE_LOCATION2 "CapturePatterns//0002.bmp"
#define FRINGE_IMAGE_LOCATION3 "CapturePatterns//0003.bmp"

#define CAPTURE_OUTPUT_DIRECTORY "CaptureImages"
#define CALIBRATION_DIRECTORY "CalibrationImages"
#define PRECAPTURED_DIRECTORY "PreCaptured"
#define RECTIFIED_DIRECTORY "RectifiedDirectory"

#define CALIBRATION_BOARD_WIDTH 800
#define CALIBRATION_BOARD_HEIGHT 600
#define DISTANCE_TO_CENTER_MM 10.0
//--------------------------------------------------------------------------------------------////


bool IsCalibration()
{
	string calibration;
	cout<<"Is this Calibration?(y/n)\n";
	cin>>calibration;

	if(strcmp(calibration.c_str(),"y")==0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool UseSaved()
{
	string savedCalibration;
	cout<<"Is this saved Calibration?(y/n)\n";
	cin>>savedCalibration;

	if(strcmp(savedCalibration.c_str(),"y")==0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void PrintHelp()
{
	cout<< "0 is left cam and 1 is right cam"<<endl;
}

bool RectifyAlreadyCapturedImages()
{
	cout << "Are the images already captured?"<<endl;
	string response;
	cin>>response;
	if(strcmp(response.c_str(),"y")==0)
	{
		return true;
	} 
	else
	{
		return false;
	}

}

int main()
{
	PrintHelp();
	bool cali = IsCalibration();

	if(cali)
	{
		cout<<"Calibration Mode.\n";
		// Check if calibration images exist
		
		if(!UseSaved())
		{			
			vector<unique_ptr<Mat[]>> calibrationImages = CaliHelper::GetCalibrationImages(NUMBER_OF_CALIBRATION_IMAGES);
			
			cvNamedWindow("bal");
			for(int i=0;i<NUMBER_OF_CALIBRATION_IMAGES;i++)
			{
				char outputNameLeft[200];
				char outputNameRight[200];

				Mat LeftImage = calibrationImages[0][i];
				Mat RightImage = calibrationImages[1][i];

				sprintf(outputNameLeft,"%s\\%d_%04d.png",CALIBRATION_DIRECTORY,0,i);
				imwrite(outputNameLeft,LeftImage);

				sprintf(outputNameLeft,"%s\\%d_%04d.png",CALIBRATION_DIRECTORY,1,i);
				imwrite(outputNameLeft,RightImage);

				#define CALIBRATION_DIRECTORY "CalibrationImages"
#define PRECALIBRATED_DIRECTORY "Precalibrate"
#define CALIBRATED_DIRECTORY "CalibratedImages"
		  
			}
		}
		else
		{
			//cvNamedWindow("debugr");
			//cvNamedWindow("debugl");
			Calibration calibration(CALIBRATION_BOARD_WIDTH,CALIBRATION_BOARD_HEIGHT,DISTANCE_TO_CENTER_MM);
			for(int i =0;i<NUMBER_OF_CALIBRATION_IMAGES;i++)
			{
				char imageNameL[200];
				char imageNameR[200];
				sprintf(imageNameL,"%s\\%d_%04d.png",CALIBRATION_DIRECTORY,0,i);
				sprintf(imageNameR,"%s\\%d_%04d.png",CALIBRATION_DIRECTORY,1,i);
				Mat leftCali = imread(imageNameL,CV_LOAD_IMAGE_GRAYSCALE );
				Mat rightCali = imread(imageNameR,CV_LOAD_IMAGE_GRAYSCALE );

				if(leftCali.empty()|| rightCali.empty())
				{
					cout<<"Calibration Image is empty.\n";	
				}
				//imshow("debugl",leftCali);
				//imshow("debugr",rightCali);
				//cvWaitKey(0);
				calibration.addImage(leftCali,rightCali,true);
			}
			calibration.processAll();
			
		}
	}
	else
	{
				if(RectifyAlreadyCapturedImages())
			{
				Calibration calibration(CALIBRATION_BOARD_WIDTH,CALIBRATION_BOARD_HEIGHT,DISTANCE_TO_CENTER_MM);
				vector<Mat> leftImages;
				vector<Mat> rightImages;

				//cvNamedWindow("debugr");
				//cvNamedWindow("debugl");

				for(int i =0;i<NUMBER_OF_CAPTURE_IMAGES;i++)
				{
					char imageNameL[200];
					char imageNameR[200];
					sprintf(imageNameL,"%s\\%d_%04d.png",PRECAPTURED_DIRECTORY,0,i);
					sprintf(imageNameR,"%s\\%d_%04d.png",PRECAPTURED_DIRECTORY,1,i);
					Mat leftCap = imread(imageNameL,CV_LOAD_IMAGE_GRAYSCALE );
					Mat rightCap = imread(imageNameR,CV_LOAD_IMAGE_GRAYSCALE );

					if(leftCap.empty()|| rightCap.empty())
					{
						cout<<"Calibration Image is empty.\n";	
					}
					
					leftImages.push_back(leftCap);
					rightImages.push_back(rightCap);
					//imshow("debugl",leftCap);
					//imshow("debugr",rightCap);
					//cvWaitKey(0);

				}
				
				vector<vector<Mat>> rectifiedImages = calibration.RectifyImages(leftImages,rightImages);
				cvNamedWindow("debug");
				for(int i =0;i<NUMBER_OF_CAPTURE_IMAGES;i++)
				{
					char imageNameL[200];
					char imageNameR[200];
					sprintf(imageNameL,"%s\\%d_%04d.png",RECTIFIED_DIRECTORY,0,i);
					sprintf(imageNameR,"%s\\%d_%04d.png",RECTIFIED_DIRECTORY,1,i);

					imwrite(imageNameL,rectifiedImages[0][i]);
					imwrite(imageNameR,rectifiedImages[1][i]);
					imshow("debug",rectifiedImages[2][i]);
					cvWaitKey(0);
				}


			}

			else
			{
				string images[3] = {FRINGE_IMAGE_LOCATION1,FRINGE_IMAGE_LOCATION2,FRINGE_IMAGE_LOCATION3};
				int numberOfImages = 3;
				unique_ptr<Mat[]> CameraImages = CaliHelper::GrabFringeImages(images,numberOfImages);

				for(int i=0;i<numberOfImages*2;i++)
				{
					int left;
					char outputName[200];
				 
					if(i%2 ==0)
					{
						left = 0;
					}
					else
					{
						left = 1;
					}
					int index = i/2;

					CameraImages[i] = CaliHelper::RotateImage(CameraImages[i],180);

					sprintf(outputName,"%s\\%d_%04d.png",CAPTURE_OUTPUT_DIRECTORY,left,index);
					imwrite(outputName,CameraImages[i]);
		  
					imshow("bal",CameraImages[i]);
					cvWaitKey(0);
				}
			}
		

		 
	}
	return 0;
}




